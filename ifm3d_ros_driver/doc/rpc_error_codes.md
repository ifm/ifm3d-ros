# XML-RPC error codes

The underlying C++ API `ifm3d` supports these error codes. They are only sent asynchronously, e.g. when configuring the camera.

| error code | description | typical solutions |
| ----- | ------- | ----- |
| 104010 | JSON syntax validation failed | Internally the sent JSON string gets checked against a JSON schema. This error happens when the parses fails  to complete. |
| 104011 | JSON does not match current schema | Internally the sent JSON string gets checked against a JSON schema. If the JSON string does not match, it will be discarded. No changes will be applied. The configuration state will be the same before trying to reconfigure. |
| 104014 | Invalid configuration | add |