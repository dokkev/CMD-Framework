"""Minimal Onshape REST client (standard library only).

Credentials come from the environment: ``ONSHAPE_ACCESS_KEY``, ``ONSHAPE_SECRET_KEY`` and optionally
``ONSHAPE_API`` (default ``https://cad.onshape.com``). API keys authenticate with HTTP basic auth.
"""

from __future__ import annotations

import base64
import json
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from typing import Any

_ACCEPT = "application/json;charset=UTF-8; qs=0.09"
API_VERSION = "v10"


class OnshapeError(RuntimeError):
    pass


@dataclass(frozen=True)
class ElementRef:
    """An element in a workspace: ``/d/{did}/w/{wid}/e/{eid}``."""

    did: str
    wid: str
    eid: str

    def path(self) -> str:
        return f"d/{self.did}/w/{self.wid}/e/{self.eid}"

    def with_workspace(self, wid: str) -> ElementRef:
        return ElementRef(self.did, wid, self.eid)

    def sibling(self, eid: str) -> ElementRef:
        return ElementRef(self.did, self.wid, eid)


class OnshapeClient:
    def __init__(self, access_key: str | None = None, secret_key: str | None = None, base_url: str | None = None):
        access_key = access_key or os.environ.get("ONSHAPE_ACCESS_KEY")
        secret_key = secret_key or os.environ.get("ONSHAPE_SECRET_KEY")
        if not (access_key and secret_key):
            raise OnshapeError("set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY")
        base = base_url or os.environ.get("ONSHAPE_API") or "https://cad.onshape.com"
        parsed = urllib.parse.urlparse(base)
        self.base_url = f"{parsed.scheme}://{parsed.netloc}/api/{API_VERSION}"
        token = base64.b64encode(f"{access_key}:{secret_key}".encode()).decode()
        self._auth = f"Basic {token}"

    def request(self, method: str, path: str, body: Any = None, query: dict | None = None, retries: int = 3) -> Any:
        url = f"{self.base_url}/{path.lstrip('/')}"
        if query:
            url += "?" + urllib.parse.urlencode(query)
        data = None if body is None else json.dumps(body).encode()
        headers = {"Authorization": self._auth, "Accept": _ACCEPT}
        if data is not None:
            headers["Content-Type"] = _ACCEPT
        for attempt in range(retries):
            req = urllib.request.Request(url, data=data, method=method, headers=headers)
            try:
                with urllib.request.urlopen(req, timeout=60) as resp:
                    raw = resp.read()
                    return json.loads(raw) if raw else None
            except urllib.error.HTTPError as err:
                if err.code == 429 and attempt + 1 < retries:
                    time.sleep(float(err.headers.get("Retry-After", 2)))
                    continue
                detail = err.read().decode(errors="replace")[:2000]
                raise OnshapeError(f"{method} {path} -> HTTP {err.code}: {detail}") from None
        raise OnshapeError(f"{method} {path}: too many retries")

    def get(self, path: str, **query) -> Any:
        return self.request("GET", path, query=query or None)

    def post(self, path: str, body: Any = None, **query) -> Any:
        return self.request("POST", path, body=body if body is not None else {}, query=query or None)

    # Documents, versions and workspaces
    def document(self, did: str) -> dict:
        return self.get(f"documents/{did}")

    def elements(self, did: str, wid: str) -> list[dict]:
        return self.get(f"documents/d/{did}/w/{wid}/elements")

    def workspaces(self, did: str) -> list[dict]:
        return self.get(f"documents/d/{did}/workspaces")

    def versions(self, did: str) -> list[dict]:
        return self.get(f"documents/d/{did}/versions")

    def create_version(self, did: str, wid: str, name: str, description: str = "") -> dict:
        return self.post(f"documents/d/{did}/versions", {"documentId": did, "workspaceId": wid, "name": name,
                                                          "description": description})

    def create_workspace(self, did: str, version_id: str, name: str, description: str = "") -> dict:
        return self.post(f"documents/d/{did}/workspaces", {"documentId": did, "versionId": version_id,
                                                            "name": name, "description": description})

    # Variable Studios
    def variables(self, ref: ElementRef) -> list[dict]:
        tables = self.get(f"variables/{ref.path()}/variables")
        return [v for table in tables for v in table["variables"]]

    def set_variables(self, ref: ElementRef, variables: list[dict]) -> None:
        """Replace the studio's variables. Each item: ``{name, expression, type?, description?}``."""
        self.post(f"variables/{ref.path()}/variables",
                  [{"type": "LENGTH", "description": "", **v} for v in variables])

    def create_variable_studio(self, did: str, wid: str, name: str) -> dict:
        return self.post(f"variables/d/{did}/w/{wid}/variablestudio", {"name": name})

    def variable_studio_references(self, ref: ElementRef) -> list[dict]:
        return self.get(f"variables/{ref.path()}/variablestudioreferences")["references"]

    def set_variable_studio_references(self, ref: ElementRef, references: list[dict]) -> None:
        self.post(f"variables/{ref.path()}/variablestudioreferences", {"references": references})

    # Assemblies
    def assembly_definition(self, ref: ElementRef, **query) -> dict:
        query = {"includeMateFeatures": "true", "includeMateConnectors": "true", **query}
        return self.get(f"assemblies/{ref.path()}", **query)

    def assembly_features(self, ref: ElementRef) -> dict:
        return self.get(f"assemblies/{ref.path()}/features")

    def mate_values(self, ref: ElementRef) -> list[dict]:
        """Every mate's current value (``rotationZ`` in radians for a revolute), with its owner occurrence."""
        return self.get(f"assemblies/{ref.path()}/matevalues").get("mateValues", [])

    def add_assembly_feature(self, ref: ElementRef, feature: dict) -> dict:
        return self.post(f"assemblies/{ref.path()}/features", {"feature": feature})

    def update_assembly_feature(self, ref: ElementRef, feature: dict) -> dict:
        """Caution: a mate with inline mate connectors fails to regenerate after this call, even when
        ``feature`` is unchanged (observed 2026-11-08). Delete and re-add such mates instead."""
        fid = urllib.parse.quote(feature["featureId"], safe="")
        return self.post(f"assemblies/{ref.path()}/features/featureid/{fid}", {"feature": feature})

    def delete_assembly_feature(self, ref: ElementRef, feature_id: str) -> None:
        self.request("DELETE", f"assemblies/{ref.path()}/features/featureid/{urllib.parse.quote(feature_id, safe='')}")
