#!/usr/bin/env python3
"""测试 run.sh 的实际 Qt 构建片段；不编译代码、不加载驱动、不访问设备。"""
from pathlib import Path
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root/'run.sh').read_text(encoding='utf-8-sig')
start = source.index('cmake -S "$project_dir"')
end = source.index('if (( build_only )); then',start)
block = source[start:end]

for case in ('normal', 'empty_binary', 'empty_object', 'still_empty', 'not_executable'):
    with tempfile.TemporaryDirectory(prefix='pcie-launcher-test-') as tmp:
        base=Path(tmp); build=base/'build'; obj=build/'CMakeFiles/pcie_viewer.dir/main.cpp.o'
        obj.parent.mkdir(parents=True)
        exe=build/'pcie_viewer'
        exe.write_bytes(b'' if case in ('empty_binary','still_empty') else b'valid executable')
        exe.chmod(0o644 if case=='not_executable' else 0o755)
        obj.write_bytes(b'' if case=='empty_object' else b'valid object')
        repair = 'true' if case in ('empty_binary','empty_object') else 'false'
        stub = r'''
set -euo pipefail
die() { echo "ERROR:$*" >&2; exit 1; }
cmake() {
    if [[ "$1" == --build ]]; then
        printf '%s\n' "$@" > "$build_dir/arguments"
        if '''+repair+r''' && [[ " $* " == *' --clean-first '* ]]; then
            printf 'rebuilt executable' > "$build_dir/pcie_viewer"
            printf 'rebuilt object' > "$build_dir/CMakeFiles/pcie_viewer.dir/main.cpp.o"
            chmod +x "$build_dir/pcie_viewer"
        fi
    fi
}
'''
        script = stub+'\nproject_dir='+shlex.quote(str(base))+'\nbuild_dir='+shlex.quote(str(build))+'\njobs=2\n'+block
        result = subprocess.run(['bash','-c',script],capture_output=True,text=True)
        should_fail = case in ('still_empty','not_executable')
        assert (result.returncode!=0)==should_fail,(case,result.stdout,result.stderr)
        args=(build/'arguments').read_text()
        assert ('--clean-first' in args)==(case in ('empty_binary','empty_object','still_empty')),case
        if not should_fail: assert exe.stat().st_size>0
print('PASS: launcher handles normal, empty binary/object and invalid build outputs')
