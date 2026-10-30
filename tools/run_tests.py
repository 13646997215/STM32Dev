"""Compile and execute the actual portable C modules. No hardware I/O."""
from pathlib import Path
import argparse, json, os, subprocess, sys, time, unittest
ROOT=Path(__file__).resolve().parents[1]
TCC=Path(os.environ.get('COURSE_HOST_CC',str(ROOT.parents[1]/'environment/tcc/win32/tcc.exe')))
def execute(exe):
    """Run one freshly compiled host test, tolerating a transient Windows lock.

    Right after the compiler writes the .exe, antivirus/indexing can hold it for
    a few milliseconds and CreateProcess fails with WinError 5. That happened
    intermittently on this machine while a plain manual run of the same .exe
    always passed, so retry once instead of failing the whole suite. Only an
    access-denied spawn is retried; a non-zero exit is always a real result.
    """
    for attempt in (1,2):
        try:
            return subprocess.run([str(exe)],capture_output=True,text=True,timeout=30)
        except PermissionError:
            if attempt==2:raise
            time.sleep(.3)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--group',default='all');ap.add_argument('--label',default='latest');ap.add_argument('--red',action='store_true');args=ap.parse_args()
    if not TCC.is_file():raise SystemExit('Set COURSE_HOST_CC to a working host C compiler; ARM GCC cannot run Windows tests.')
    out=ROOT/'build'/'host';out.mkdir(parents=True,exist_ok=True)
    groups=sorted((ROOT/'tests/host').glob('test_*.c'))
    if args.group!='all':groups=[p for p in groups if p.stem=='test_'+args.group]
    if not groups:raise SystemExit('No tests matched')
    results=[]
    sources=sorted((ROOT/'App/Src').glob('*.c'))
    for test in groups:
        exe=out/(test.stem+'.exe');use=sources[:]
        if args.red:use=[ROOT/'tests/host'/('missing_'+args.group+'.c')]
        if test.stem=='test_runtime' and not args.red:use+=[ROOT/'BSP/Src/runtime.c']
        cmd=[str(TCC),'-Wall','-I'+str(ROOT/'App/Inc'),'-I'+str(ROOT/'BSP/Inc'),'-I'+str(ROOT/'tests/host'),str(test),*[str(p) for p in use],'-o',str(exe)]
        build=subprocess.run(cmd,capture_output=True,text=True)
        run=execute(exe) if build.returncode==0 else None
        r={'test':test.name,'command':cmd,'compile_exit':build.returncode,'compile_output':build.stdout+build.stderr,'run_exit':run.returncode if run else None,'output':run.stdout+run.stderr if run else ''};results.append(r)
        print(test.name, 'COMPILE_ERROR' if not run else ('PASS' if run.returncode==0 else 'FAIL'));print(r['compile_output']+r['output'],end='')
    target=ROOT/'test-results'/(args.label+'.json');target.parent.mkdir(exist_ok=True)
    target.write_text(json.dumps(results,indent=2),encoding='utf-8')
    return 0 if all(r['compile_exit']==0 and r['run_exit']==0 for r in results) else 1
if __name__=='__main__':sys.exit(main())
