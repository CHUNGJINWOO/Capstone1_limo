"""Small shared helpers for DDS graph evidence (Phase 3.5, 2026-11-21). Graph evidence is consistency, not authentication."""


def gid_hex(info, *, require_nonzero=False):
    """Hex string of an endpoint GID, or None when it cannot be read. With require_nonzero, an empty / all-zero GID is None too."""
    try:
        g = bytes(info.endpoint_gid).hex()
    except Exception:
        return None
    if require_nonzero and not (g and any(bytes.fromhex(g))):
        return None
    return g


def participant_prefix(gid):
    """First 12 bytes of an rmw_fastrtps GID = the DDS participant GUID prefix (observed, not a documented guarantee)."""
    return gid[:24] if isinstance(gid, str) and len(gid) >= 24 else None


def endpoints(query, topic):
    """[(namespace, node_name)] of the endpoints the query returns for `topic`."""
    return [(x.node_namespace, x.node_name) for x in query(topic)]
