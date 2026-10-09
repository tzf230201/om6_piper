"""Pure scan timing profiles; no robot connection or motion calls."""
def timing_profile(segments,name):
    if name not in ('original','vision_slow'):raise ValueError('Unknown profile')
    factor=2.0 if name=='vision_slow' else 1.0
    return ([{**s,'offset_deg':list(s['offset_deg']),'duration':s['duration']*factor} for s in segments],
            3.0 if name=='vision_slow' else 1.0,
            10 if name=='vision_slow' else 20)
