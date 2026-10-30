"""Software-only verification entry point. No serial ports, probes, flashing or motion."""
from pathlib import Path
import argparse,hashlib,json,os,re,subprocess,sys,time
ROOT=Path(__file__).resolve().parents[1]
BASE=Path(os.environ.get('COURSE_BASELINE_DIR',str(ROOT.parent/'F401_MOTOR_BASIC')))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--no-build',action='store_true');args=ap.parse_args();results=[]
    commands=[['tools/run_tests.py','--label','final-c'],['-m','unittest','discover','-s','tests/python','-v'],['PC/monitor.py','--demo','--smoke']]
    if not args.no_build:commands += [['tools/build.py'],['tools/check_artifacts.py']]
    for command in commands:
        cmd=[sys.executable,'-X','utf8',*command];r=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,errors='replace',timeout=180)
        results.append({'command':cmd,'exit':r.returncode,'output':r.stdout+r.stderr});print('PASS' if r.returncode==0 else 'FAIL', ' '.join(command));print(r.stdout+r.stderr)
    baseline=json.loads((ROOT/'docs/baseline-sha256.json').read_text());changed=[]
    for rel,expected in baseline.items():
        p=BASE/rel
        if not p.is_file() or sha(p)!=expected:changed.append(rel)
    vendor=[]
    for folder in ['Drivers','Middlewares']:
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and (not (BASE/p.relative_to(ROOT)).is_file() or sha(p)!=sha(BASE/p.relative_to(ROOT))):vendor.append(str(p.relative_to(ROOT)))
    outside=[]
    for rel in ['Core/Src/main.c','Core/Src/freertos.c','Core/Src/stm32f4xx_it.c','Core/Inc/FreeRTOSConfig.h']:
        def strip(path):
            text=path.read_text(encoding='utf-8')
            return re.sub(r'(/\* USER CODE BEGIN ([^*]+) \*/).*?(/\* USER CODE END \2 \*/)',r'\1\3',text,flags=re.S)
        if strip(ROOT/rel)!=strip(BASE/rel):outside.append(rel)
    security=[]
    patterns=[r'\beval\s*\(',r'\bexec\s*\(',r'shell\s*=\s*True',r'-----BEGIN (?:RSA |EC )?PRIVATE KEY-----',r'AKIA[A-Z0-9]{16}']
    for folder in ['App','BSP','PC']:
        for p in (ROOT/folder).rglob('*'):
            if p.suffix not in ('.c','.h','.py'):continue
            text=p.read_text(encoding='utf-8')
            if any(re.search(pattern,text) for pattern in patterns):security.append(str(p.relative_to(ROOT)))
    evidence={'baseline_checked':len(baseline),'baseline_changed':changed,'vendor_changed':vendor,'core_outside_user_code_changed':outside,'security_findings':security}
    for key,value in evidence.items():print(key,value)
    ok=all(r['exit']==0 for r in results) and not any([changed,vendor,outside,security])
    source_hashes={}
    for folder in ['App','BSP','Core','Drivers','Middlewares','cmake','PC','.vscode','.settings']:
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':source_hashes[p.relative_to(ROOT).as_posix()]=sha(p)
    for name in ['CMakeLists.txt','CMakePresets.json','STM32F401CCUX_FLASH.ld']:
        source_hashes[name]=sha(ROOT/name)
    artifact_hashes={}
    if not args.no_build:
        for preset in ['Debug','Release']:
            for suffix in ['elf','bin','hex','map']:
                p=ROOT/f'build/{preset}/F401_COURSE.{suffix}';artifact_hashes[p.relative_to(ROOT).as_posix()]=sha(p)
    report={'software_only':True,'hardware_accessed':False,'success':ok,'cross_build_requested':not args.no_build,'results':results,'checks':evidence,'source_hashes':source_hashes,'artifact_hashes':artifact_hashes}
    (ROOT/'test-results/verification.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return 0 if ok else 1
if __name__=='__main__':sys.exit(main())
