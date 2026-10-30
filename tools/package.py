"""Package only a verified software state. Never overwrites an existing archive."""
from pathlib import Path
import hashlib,json,sys,zipfile
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report=json.loads((ROOT/'test-results/verification.json').read_text())
    if not report['success'] or not report['cross_build_requested']:raise SystemExit('Run full tools/verify.py first')
    for name,value in {**report['source_hashes'],**report['artifact_hashes']}.items():
        if sha(ROOT/name)!=value:raise SystemExit('Changed since verification: '+name)
    folder=ROOT.parents[1]/'exports';folder.mkdir(parents=True,exist_ok=True)
    stem='F401_COURSE_software';index=1
    while (folder/f'{stem}_{index:02d}.zip').exists():index+=1
    archive=folder/f'{stem}_{index:02d}.zip';files={}
    allowed_build=set(report['artifact_hashes'])
    for p in ROOT.rglob('*'):
        if not p.is_file():continue
        rel=p.relative_to(ROOT);name=rel.as_posix()
        if '__pycache__' in rel.parts or '.git' in rel.parts or p.suffix=='.pyc':continue
        if rel.parts[0]=='build' and name not in allowed_build:continue
        files[name]=p
    manifest={name:sha(p) for name,p in sorted(files.items())}
    with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
        for name,p in sorted(files.items()):z.write(p,'F401_COURSE/'+name)
        z.writestr('F401_COURSE/manifest-sha256.json',json.dumps(manifest,indent=2))
    with zipfile.ZipFile(archive) as z:
        if z.testzip() is not None:raise SystemExit('ZIP integrity failure')
        for name,value in manifest.items():
            if hashlib.sha256(z.read('F401_COURSE/'+name)).hexdigest()!=value:raise SystemExit('Archive hash mismatch: '+name)
    record={'archive':str(archive),'sha256':sha(archive),'files':len(files),'zip_entries_verified':True,'hardware_tested':False}
    archive.with_suffix('.json').write_text(json.dumps(record,indent=2),encoding='utf-8');print(json.dumps(record,indent=2))
    return 0
if __name__=='__main__':sys.exit(main())
