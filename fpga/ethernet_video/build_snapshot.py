"""Package the exact PDS source inputs and the current bitstream."""

import csv
import hashlib
import re
import sys
import zipfile
from pathlib import Path


project = Path(sys.argv[1]).resolve()
output = Path(sys.argv[2]).resolve()
output.mkdir(parents=True, exist_ok=True)
pds = project / "3_ddr_test.pds"
text = pds.read_text(encoding="utf-8", errors="replace")
inputs = set(re.findall(r'\(_(?:file|ip|ip_source_item) "((?:source|ipcore)/[^"]+)"', text))
paths = sorted(inputs | {
    "3_ddr_test.pds",
    "ddr_test.fdc",
    "generate_bitstream/test_ddr.sbit",
})
missing = [path for path in paths if not (project / path).is_file()]
if missing:
    raise SystemExit("Missing PDS inputs: " + ", ".join(missing))

archive = output / "ethernet_video_fpga_20260916.zip"
manifest = output / "SOURCE_MANIFEST.csv"
rows = []
with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
    for rel in paths:
        data = (project / rel).read_bytes()
        zf.writestr(rel, data)
        rows.append((rel, len(data), hashlib.sha256(data).hexdigest()))
with manifest.open("w", encoding="utf-8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(("path", "bytes", "sha256"))
    writer.writerows(rows)
with zipfile.ZipFile(archive) as zf:
    assert zf.testzip() is None
    for rel, size, digest in rows:
        data = zf.read(rel)
        assert len(data) == size and hashlib.sha256(data).hexdigest() == digest
print(f"{archive}: {archive.stat().st_size} bytes, {len(rows)} verified files")
print(f"{manifest}: {manifest.stat().st_size} bytes")
