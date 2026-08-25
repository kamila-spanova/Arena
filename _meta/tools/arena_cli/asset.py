"""Inspect and move resolvable arena data. KINDS below is the list of what is movable."""

from __future__ import annotations

import dataclasses
import importlib
import sys

from common import CLIError, make_verb
from complete import Manifest, Nothing, Static, Sub, Union

_TREE = "arena_simulation_setup.tree"
_BENCH = "arena_evaluation.benchmark.tree"


@dataclasses.dataclass(frozen=True)
class Kind:
    """One resolvable data kind: which identifier answers for it, and how it publishes."""

    description: str
    module: str
    identifier: str
    providers: str  # bucket-list symbol on arena_simulation_setup.tree
    parse: bool = False  # built via .parse() rather than the constructor
    bundle: bool = False  # resolves to a yaml, so publishing wraps it into a directory
    closure: str | None = None  # key into _PREFLIGHTS, checked before publishing
    repo_subdir: str | None = None  # ships in the repo under ASS_DIR/<subdir>
    complete_key: str | None = None  # shell-completion manifest key


KINDS = {
    "object": Kind("3D object models", f"{_TREE}.assets.Object", "ObjectIdentifier", "NETWORK_PROVIDERS", parse=True),
    "human": Kind("human models", f"{_TREE}.assets.Human", "HumanIdentifier", "NETWORK_PROVIDERS", parse=True),
    "material": Kind("surface materials", f"{_TREE}.assets.Material", "MaterialIdentifier", "NETWORK_PROVIDERS", parse=True),
    "wall": Kind("wall kinds", f"{_TREE}.Wall", "WallIdentifier", "NETWORK_PROVIDERS", parse=True),
    "world": Kind("worlds", f"{_TREE}.World", "WorldIdentifier", "WORLD_PROVIDERS", closure="world", repo_subdir="worlds", complete_key="world"),
    "suite": Kind("benchmark suites", _BENCH, "SuiteIdentifier", "BENCHMARK_PROVIDERS", bundle=True, closure="suite"),
    "contest": Kind("benchmark contests", _BENCH, "ContestIdentifier", "BENCHMARK_PROVIDERS", bundle=True),
    "manifest": Kind("report manifests", _BENCH, "ManifestIdentifier", "BENCHMARK_PROVIDERS", bundle=True),
}

_VALUED_FLAGS = ("--bucket",)


def _positionals(argv: list[str]) -> list[str]:
    """Bare arguments. Skips flags, and the value a valued flag consumes, so that
    `--bucket B` ahead of the positionals is not read as the kind."""
    out: list[str] = []
    skip = False
    for arg in argv:
        if skip:
            skip = False
        elif arg in _VALUED_FLAGS:
            skip = True
        elif not arg.startswith("-"):
            out.append(arg)
    return out


def _kind(kind: str) -> Kind:
    try:
        return KINDS[kind]
    except KeyError:
        raise CLIError(f"unknown kind {kind!r}, expected one of {', '.join(sorted(KINDS))}") from None


def _identifier_type(kind: str):
    """The identifier class for one kind. Imports the ROS-side package lazily."""
    spec = _kind(kind)
    return getattr(importlib.import_module(spec.module), spec.identifier)


def _identifier(kind: str, name: str):
    identifier_t = _identifier_type(kind)
    return identifier_t.parse(name) if _kind(kind).parse else identifier_t(name)


def find_main(argv: list[str]) -> int:
    """Show every resolver's verdict for one asset, without downloading it.

    `arena asset find <kind> <name> [--json]`.
    """
    as_json = "--json" in argv
    args = _positionals(argv)
    if len(args) != 2:
        raise CLIError("find takes KIND and NAME")
    kind, name = args

    from arena_simulation_setup.tree import Verdict

    labels = {Verdict.HIT: "HIT", Verdict.MISS: "miss", Verdict.SHADOWED: "shadowed"}
    verdicts = _identifier(kind, name).probe_sync()

    if as_json:
        import json

        print(
            json.dumps(
                [
                    {"resolver": repr(v.resolver), "verdict": v.verdict.value, "path": str(v.path) if v.path else None}
                    for v in verdicts
                ],
                indent=2,
            )
        )
        return 0

    print(f"{kind} {name}")
    width = len(str(len(verdicts)))
    for n, verdict in enumerate(verdicts, start=1):
        label = labels[verdict.verdict]
        suffix = f"  {verdict.path}" if verdict.path is not None else ""
        print(f"  {n:>{width}d}  {repr(verdict.resolver):<64s} {label}{suffix}")
    if not any(v.verdict is Verdict.HIT for v in verdicts):
        return 1
    return 0


def ls_main(argv: list[str]) -> int:
    """List every available asset of one kind.

    `arena asset ls <kind> [--network]`. With --network, bucket-only entries are
    listed too and names that a local copy shadows are marked.
    """
    network = "--network" in argv
    args = _positionals(argv)
    if len(args) != 1:
        raise CLIError("ls takes KIND")
    kind = args[0]

    from arena_simulation_setup.tree import NetResolver

    identifier_t = _identifier_type(kind)

    if not network:
        for name in sorted({identifier.shortname for identifier in identifier_t.listall()}):
            print(name)
        return 0

    # a NetResolver's own cache is not a competing source, so it must not count as local
    local = {
        identifier.shortname
        for resolver in identifier_t._resolvers
        if not isinstance(resolver, NetResolver)
        for identifier in resolver.listall()
    }
    remote = {
        identifier.shortname
        for resolver in identifier_t._resolvers
        if isinstance(resolver, NetResolver)
        for identifier in resolver.listall(network=True)
    }
    for name in sorted(local | remote):
        marker = "  shadowed" if name in local and name in remote else ""
        print(f"{name}{marker}")
    return 0


def _buckets(kind: str) -> list[str]:
    from arena_simulation_setup import tree

    return list(getattr(tree, _kind(kind).providers))


def _bucket_of(argv: list[str], kind: str) -> str:
    for i, arg in enumerate(argv):
        if arg == "--bucket" and i + 1 < len(argv):
            return argv[i + 1]
    candidates = _buckets(kind)
    if not candidates:
        raise CLIError(f"no bucket configured for {kind}, pass --bucket")
    return candidates[0]


def _cache_root(bucket: str):
    from arena_simulation_setup import ARENA_ASSETS_DIR

    return ARENA_ASSETS_DIR / bucket


def _net(bucket: str, *args: str) -> int:
    import subprocess

    return subprocess.call(["ros2", "run", "arena_models", "arena_models", "-s", "net", bucket, *args])


def pull_main(argv: list[str]) -> int:
    """Download an asset from its bucket into the local cache.

    `arena asset pull <kind> <name> [--bucket B]`.
    """
    args = _positionals(argv)
    if len(args) < 2:
        raise CLIError("pull takes KIND and NAME")
    kind, name = args[0], args[1]
    bucket = _bucket_of(argv, kind)
    identifier = _identifier(kind, name)
    return _net(bucket, "fetch", str(identifier.relpath()), "-o", str(_cache_root(bucket)))


def _world_dangling(view, source) -> list[str]:
    """World references that would dangle once published: resolvable only from a local-only source."""
    from arena_simulation_setup.tree import DynamicPaths, DynamicPathResolver, NetResolver

    # world-local assets only resolve once WORLD points at the world being published
    DynamicPaths.WORLD.path = source

    dangling: list[str] = []
    for identifier in view.identifiers(strict=True):
        # only the winning resolver matters here, and resolve_source stops at it rather
        # than probing every remaining one over the network
        try:
            hit = identifier.resolve_source_sync()
        except FileNotFoundError:
            dangling.append(f"{identifier.shortname} (unresolvable)")
            continue
        if isinstance(hit.resolver, NetResolver):
            continue
        if isinstance(hit.resolver, DynamicPathResolver) and hit.path.is_relative_to(source):
            continue
        dangling.append(f"{identifier.shortname} (only at {hit.path})")
    return dangling


def _suite_dangling(suite, source) -> list[str]:
    """Worlds a suite stages that would not resolve for someone else. One bundled under the
    suite counts, since the runner exports it via ARENA_WORLD_PATH.

    Task modes are deliberately not checked: they are code rather than data shipped with the
    suite, so the publisher's registry says nothing about the consumer's, and PROMPT is not
    registered until a human simulator constructs it.
    """
    from arena_simulation_setup.tree.World import WorldIdentifier

    dangling: list[str] = []
    for world in dict.fromkeys(stage.map for stage in suite.stages):
        if (source / "worlds" / world).is_dir():
            continue
        try:
            WorldIdentifier(world).resolve_source_sync()
        except FileNotFoundError:
            dangling.append(f"world {world} (unresolvable)")
    return dangling


_PREFLIGHTS = {"world": _world_dangling, "suite": _suite_dangling}


def _config_source(identifier):
    """Where a bundled kind lives. It resolves to its yaml, so a directory bundle is
    reported as the directory holding it and a flat config as the file itself."""
    resolved = identifier.resolve_source_sync().path
    if resolved.is_file() and resolved.parent.name == identifier.name:
        return resolved.parent
    return resolved


def _drop_listing(kind: str, bucket: str) -> None:
    """A publish makes the cached bucket listing stale, so the next ls re-fetches it."""
    from arena_simulation_setup.tree import NetResolver

    for resolver in _identifier_type(kind)._resolvers:
        if isinstance(resolver, NetResolver) and resolver._provider == bucket:
            resolver._listing_path.unlink(missing_ok=True)


def push_main(argv: list[str]) -> int:
    """Publish an asset to its bucket.

    `arena asset push <kind> <name> [--bucket B] [--force] [--yes]`. Kinds carrying a
    closure are checked first, and publishing is refused if any reference would not resolve
    for someone else. A flat benchmark config is wrapped into the directory bundle the
    bucket requires.
    """
    import shutil
    import tempfile
    from pathlib import Path

    args = _positionals(argv)
    if len(args) < 2:
        raise CLIError("push takes KIND and NAME")
    kind, name = args[0], args[1]
    spec = _kind(kind)
    force = "--force" in argv
    bucket = _bucket_of(argv, kind)

    identifier = _identifier(kind, name)

    if spec.repo_subdir is not None and not force:
        from arena_simulation_setup import ASS_DIR

        if (ASS_DIR / spec.repo_subdir / name).is_dir():
            print(f"{name} ships in the repo, so a published copy would never be read. Use --force to publish anyway.", file=sys.stderr)
            return 1

    loaded = identifier.resolve_sync()
    source = _config_source(identifier) if spec.bundle else loaded.path

    verdict = "publishing anyway" if force else f"refusing to publish {name}"
    check = _PREFLIGHTS.get(spec.closure)
    dangling: list[str] = []
    if check is not None:
        try:
            dangling = check(loaded, source)
        except Exception as exc:
            # a strict walk raises on a reference it cannot read, and that must never be
            # mistaken for an asset that simply has no references
            reason = str(exc).splitlines()[0] or type(exc).__name__
            print(f"{verdict}: references could not be enumerated ({reason})", file=sys.stderr)
            if not force:
                return 1
    if dangling:
        print(f"{verdict}: {len(dangling)} reference(s) would not resolve for anyone else", file=sys.stderr)
        for entry in dangling:
            print(f"  MISS  {entry}", file=sys.stderr)
        if not force:
            print("bundle them under the asset directory, or pass --force", file=sys.stderr)
            return 1

    if "--yes" not in argv:
        print(f"publish {kind} {name} to gs://{bucket}/{identifier.relpath()} ? [y/N] ", end="", flush=True)
        if input().strip().lower() not in ("y", "yes"):
            return 1

    with tempfile.TemporaryDirectory() as staging:
        staged = Path(staging) / name
        if source.is_dir():
            # .ttl is cache bookkeeping; publishing it would ship a stale freshness stamp
            shutil.copytree(source, staged, ignore=shutil.ignore_patterns(".ttl"))
        else:
            staged.mkdir(parents=True)
            shutil.copy(source, staged / f"{kind}.yaml")
        result = _net(bucket, "author", str(staged), "-d", str(identifier.relpath()))
    if result == 0:
        _drop_listing(kind, bucket)
    return result


_SUB = {
    "find": find_main,
    "ls": ls_main,
    "pull": pull_main,
    "push": push_main,
}

_KIND_SPEC = Sub({kind: (Manifest(spec.complete_key) if spec.complete_key else Nothing()) for kind, spec in KINDS.items()})

COMPLETE = Sub(
    {
        "find": _KIND_SPEC,
        "ls": Union(Static({kind: spec.description for kind, spec in KINDS.items()})),
        "pull": _KIND_SPEC,
        "push": _KIND_SPEC,
    }
)


def asset_main(argv: list[str]) -> int:
    """Inspect and move resolvable arena data.

    `arena asset find <kind> <name>` shows every resolver's verdict, marking which
    source won and which were shadowed. `arena asset ls <kind>` lists what is available.
    """
    if not argv or argv[0] not in _SUB:
        raise CLIError(f"asset takes one of: {', '.join(_SUB)}")
    return _SUB[argv[0]](argv[1:]) or 0


VERB = make_verb("asset", asset_main, passthrough=True, complete=COMPLETE)


if __name__ == "__main__":
    sys.exit(asset_main(sys.argv[1:]))
