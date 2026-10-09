"""Offline/viewer job prototype over the native reader, not a camera-thread API.

One bounded native region per step. No transaction is held between steps.
Completion needs a stable durable revision; changed revisions abort, never mix.
"""
import itertools
import time
import numpy as np


class RegionalJob:
    def __init__(self,reader,lower,upper,node_cap=40000,edge_cap=200000,byte_cap=32*1024**2):
        bounds=np.asarray([lower,upper])
        if bounds.shape!=(2,3) or bounds.dtype.kind not in 'iu' or np.any(bounds>1000000000) or (bounds.dtype.kind=='i' and np.any(bounds < -1000000000)):
            raise ValueError('Need bounded integer coordinates')
        self.reader=reader;self.lower=bounds[0].astype('int64');self.upper=bounds[1].astype('int64')
        widths=self.upper-self.lower
        if self.lower.shape!=(3,) or self.upper.shape!=(3,) or np.any(widths<1) or np.any(widths>64) or np.prod(widths)>64:
            raise ValueError('Job requires 1..64 regions')
        if not 1<=node_cap<=40000 or not 1<=edge_cap<=200000 or not 1<=byte_cap<=32*1024**2:
            raise ValueError('Invalid job retention budget')
        self.regions=list(itertools.product(*(range(int(a),int(b)) for a,b in zip(self.lower,self.upper))))
        self.parts=[];self.revision=None;self.failed=False;self.complete=False;self.graph=None
        self.node_cap=node_cap;self.edge_cap=edge_cap;self.byte_cap=byte_cap
        self.nodes=0;self.owned_edges=0;self.retained_bytes=0;self.steps=[];self.assembly_ms=None

    @property
    def ready_to_assemble(self):return not self.failed and len(self.parts)==len(self.regions)

    def abort(self):
        self.failed=True;self.complete=False;self.parts.clear();self.graph=None;self.retained_bytes=0

    def step(self):
        if self.failed or self.complete or self.ready_to_assemble:raise RuntimeError('Job not accepting another step')
        r=np.asarray(self.regions[len(self.parts)]);start=time.perf_counter()
        try:
            g,s=self.reader.query(r,r+1,regions=1)
            revision=(s['durable_batch'],s['durable_frames'],s['durable_source'])
            if self.revision is not None and revision!=self.revision:raise RuntimeError('Archive revision changed; restart query job')
            self.revision=revision
            nodes=len(g['points']);edges=len(g['edges'])+len(g['boundary_keys']);size=sum(v.nbytes for v in g.values())
            if self.nodes+nodes>self.node_cap or self.owned_edges+edges>self.edge_cap or self.retained_bytes+size>self.byte_cap:
                raise RuntimeError('Job retention budget exceeded')
            self.nodes+=nodes;self.owned_edges+=edges;self.retained_bytes+=size;self.parts.append(g)
            row=dict(region=r.tolist(),stats=s,step_ms=(time.perf_counter()-start)*1000,completed_regions=len(self.parts),
                     retained_array_bytes=self.retained_bytes)
            self.steps.append(row);return row
        except Exception:self.abort();raise

    def assemble(self):
        # This may exceed a display-frame budget. Call from a consumer worker, not camera capture.
        if not self.ready_to_assemble:raise RuntimeError('Incomplete/failed job cannot produce a complete graph')
        start=time.perf_counter()
        try:
            node_fields=('points','normals','radii','seen','keys','node_witness')
            result={k:np.concatenate([g[k] for g in self.parts],axis=0) for k in node_fields}
            # Lexicographic joins in NumPy's compiled loops, instead of one Python
            # tuple/dictionary operation per endpoint. Integer keys stay exact.
            dtype=np.dtype([(f'k{i}','<i8') for i in range(4)])
            def key_view(a):return np.ascontiguousarray(a,dtype='<i8').view(dtype).reshape(-1)
            keys=key_view(result['keys']);order=np.argsort(keys,kind='stable');sorted_keys=keys[order]
            if np.any(sorted_keys[1:]==sorted_keys[:-1]):raise RuntimeError('Overlapping region nodes')
            pair_chunks=[];seen_chunks=[];witness_chunks=[];offset=0
            for g in self.parts:
                pair_chunks.append(g['edges']+offset);seen_chunks.append(g['edge_seen']);witness_chunks.append(g['edge_witness'])
                offset+=len(g['points'])
            boundary=np.concatenate([g['boundary_keys'] for g in self.parts]);other=boundary[:,4:7]//8
            selected=np.all((other>=self.lower)&(other<self.upper),axis=1);boundary=boundary[selected]
            def locate(a):
                lookup=key_view(a);indices=np.searchsorted(sorted_keys,lookup)
                if np.any(indices>=len(sorted_keys)) or np.any(sorted_keys[indices]!=lookup):
                    raise RuntimeError('Missing interior endpoint while stitching')
                return order[indices]
            seams=len(boundary)
            pair_chunks.append(np.column_stack((locate(boundary[:,:4]),locate(boundary[:,4:]))).astype('int32'))
            seen_chunks.append(np.concatenate([g['boundary_seen'] for g in self.parts])[selected])
            witness_chunks.append(np.concatenate([g['boundary_witness'] for g in self.parts])[selected])
            result.update(edges=np.concatenate(pair_chunks),edge_seen=np.concatenate(seen_chunks),edge_witness=np.concatenate(witness_chunks))
            self.graph=result;self.complete=True;self.parts.clear()
            self.assembly_ms=(time.perf_counter()-start)*1000;self.restored_boundary_edges=seams
            return result
        except Exception:self.abort();raise
