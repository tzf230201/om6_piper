"""Repository integrity and documentation checks; no application execution."""
import _bootstrap
import hashlib
import json
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET
from workspace_paths import REPO_ROOT, WORKSPACE_ROOT, repo_file, workspace_file


class RepositoryTests(unittest.TestCase):
    def test_root_contains_only_launcher_and_two_compatibility_adapters(self):
        self.assertEqual({p.name for p in REPO_ROOT.glob('*.py')},
                         {'piper.py', 'trajectory_test.py', 'print_joints.py'})

    def test_manual_hardware_examples_are_not_in_test_collection(self):
        names = {'test_can.py', 'read_joint.py', 'joint_raw_test.py',
                 'gripper_test.py', 'gripper_piper_sdk.py'}
        for name in names:
            self.assertTrue((REPO_ROOT / 'examples/manual' / name).is_file())
            self.assertFalse((REPO_ROOT / 'tests' / name).exists())

    def test_source_references_and_shared_root(self):
        self.assertEqual(repo_file('live_mapping.py'), REPO_ROOT / 'src/live_mapping.py')
        self.assertEqual(repo_file('trajectory_test.py'), REPO_ROOT / 'src/trajectory_test.py')
        self.assertEqual(repo_file('test_delta_archive.py'), REPO_ROOT / 'tests/test_delta_archive.py')
        self.assertEqual(workspace_file('om6_piper/regional_read_job.py'), REPO_ROOT / 'src/regional_read_job.py')
        self.assertEqual(workspace_file('cpp_surface/CMakeLists.txt'), WORKSPACE_ROOT / 'cpp_surface/CMakeLists.txt')
        for name in ('../file', '/file'):
            with self.assertRaises(ValueError):
                repo_file(name)

    def test_previous_readme_and_hardware_sources_preserved(self):
        original = REPO_ROOT / '.local/repository_layout_20261009/original_root'
        self.assertEqual((REPO_ROOT / 'docs/research/DEVELOPMENT_LOG.md').read_bytes(),
                         (original / 'README.md').read_bytes())
        self.assertEqual((REPO_ROOT / 'src/trajectory_test.py').read_bytes(),
                         (original / 'trajectory_test.py').read_bytes())
        for p in (REPO_ROOT / 'examples/manual').glob('*.py'):
            self.assertEqual(p.read_bytes(), (original / p.name).read_bytes(), p.name)

    def test_current_documentation_links_resolve(self):
        docs = [REPO_ROOT / 'README.md', REPO_ROOT / 'CONTRIBUTING.md',
                *(REPO_ROOT / 'docs').glob('*.md'), REPO_ROOT / 'src/README.md',
                REPO_ROOT / 'tests/README.md', REPO_ROOT / 'docs/research/README.md',
                REPO_ROOT / 'docs/images/README.md', REPO_ROOT / 'piper_readonly/README.md']
        for doc in docs:
            body = re.sub(r'```.*?```', '', doc.read_text(encoding='utf-8'), flags=re.S)
            for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)', body):
                target = target.split('#')[0]
                if not target or re.match(r'[a-z]+:', target):
                    continue
                with self.subTest(document=doc.name, target=target):
                    self.assertTrue((doc.parent / target).exists())

    def test_documentation_images_are_real_files_with_provenance(self):
        directory = REPO_ROOT / 'docs/images'
        for record in json.loads((directory / 'provenance.json').read_text()):
            self.assertEqual(hashlib.sha256((directory / record['image']).read_bytes()).hexdigest(),
                             record['image_sha256'].lower())
            self.assertEqual(hashlib.sha256((WORKSPACE_ROOT / record['source']).read_bytes()).hexdigest(),
                             record['source_sha256'].lower())
        root = ET.parse(directory / 'architecture.svg').getroot()
        self.assertEqual(root.attrib['viewBox'], '0 0 1280 630')

    def test_source_compiles_without_importing_hardware(self):
        for directory in ('src', 'piper_readonly', 'tools', 'tests'):
            for path in (REPO_ROOT / directory).glob('*.py'):
                compile(path.read_bytes(), str(path), 'exec')


if __name__ == '__main__':
    unittest.main()
