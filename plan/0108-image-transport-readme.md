# Image transport message reference

Goal: document image topic, descriptor fields and SHM ownership in the image library README.
Steps: add topic/direction/default-port/schema table, exact notification framing, field units and SHM layout/lifecycle notes.
Interfaces: none.
Verification: check against shm.py, schema links and diff whitespace.
Scope/defaults: documentation only, default camera/front/image on TCP 5556.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: checked framing, fields and ownership against shm.py; all Markdown links resolve; documentation only, runtime tests not rerun.
