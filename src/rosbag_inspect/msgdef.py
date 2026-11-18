"""Parse ROS 2 ``.msg`` definitions and keep a registry of known message types.

Type names are normalised to ``package/msg/Name``. A registry is filled from the built-in
table (:data:`BUILTIN_DEFINITIONS`) and from MCAP ``ros2msg`` schemas, which concatenate the
top-level definition and its dependencies separated by a line of 80 ``=`` characters followed
by ``MSG: package/Name``.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

PRIMITIVE_SIZES: dict[str, int] = {
    "bool": 1,
    "byte": 1,
    "char": 1,
    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "int64": 8,
    "uint64": 8,
    "float32": 4,
    "float64": 8,
}
STRING_TYPES = ("string", "wstring")
SEPARATOR = "=" * 80
EMPTY_PLACEHOLDER = "structure_needs_at_least_one_member"

_NAME_RE = re.compile(r"[A-Za-z][A-Za-z0-9_]*$")
_CONST_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)\s*=(.*)$")
_ARRAY_RE = re.compile(r"^(?P<base>[^\[\]]+)(\[(?P<bound><=)?(?P<size>[0-9]*)\])?$")


class MsgDefError(ValueError):
    """A message definition could not be parsed or resolved."""


@dataclass(frozen=True)
class FieldType:
    """Type of one field.

    ``array`` is ``""`` for a single value, ``"fixed"`` for ``T[N]`` and ``"sequence"`` for
    ``T[]`` / ``T[<=N]``; ``length`` holds N (fixed size or upper bound).
    """

    base: str
    array: str = ""
    length: int | None = None
    string_bound: int | None = None

    @property
    def is_primitive(self) -> bool:
        return self.base in PRIMITIVE_SIZES or self.base in STRING_TYPES

    def __str__(self) -> str:
        text = self.base
        if self.string_bound is not None:
            text += f"<={self.string_bound}"
        if self.array == "fixed":
            text += f"[{self.length}]"
        elif self.array == "sequence":
            text += f"[<={self.length}]" if self.length is not None else "[]"
        return text


@dataclass(frozen=True)
class Field:
    name: str
    type: FieldType


@dataclass
class MessageDefinition:
    name: str
    fields: list[Field]
    constants: dict[str, str] = field(default_factory=dict)


def normalize_type(name: str, package: str | None = None) -> str:
    """Return ``pkg/msg/Name`` for ``pkg/Name``, ``pkg/msg/Name`` or bare ``Name``."""
    name = name.strip()
    if name in ("time", "duration"):
        return f"builtin_interfaces/msg/{name.capitalize()}"
    parts = name.split("/")
    if len(parts) == 3:
        return name
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    if len(parts) == 1:
        if name == "Header":
            return "std_msgs/msg/Header"
        if package is None:
            raise MsgDefError(f"type {name!r} has no package")
        return f"{package}/msg/{name}"
    raise MsgDefError(f"bad type name {name!r}")


def _package_of(full_name: str) -> str:
    return full_name.split("/")[0]


def _parse_type(token: str, package: str) -> FieldType:
    match = _ARRAY_RE.match(token)
    if match is None:
        raise MsgDefError(f"bad field type {token!r}")
    base = match.group("base")
    string_bound: int | None = None
    if "<=" in base:
        base, _, bound = base.partition("<=")
        if base not in STRING_TYPES or not bound.isdigit():
            raise MsgDefError(f"bad bounded type {token!r}")
        string_bound = int(bound)
    if base not in PRIMITIVE_SIZES and base not in STRING_TYPES:
        base = normalize_type(base, package)
    if match.group(2) is None:
        return FieldType(base, string_bound=string_bound)
    size = match.group("size")
    if match.group("bound"):
        if not size:
            raise MsgDefError(f"bounded array without bound: {token!r}")
        return FieldType(base, "sequence", int(size), string_bound)
    if size:
        if int(size) == 0:
            raise MsgDefError(f"fixed array of size 0: {token!r}")
        return FieldType(base, "fixed", int(size), string_bound)
    return FieldType(base, "sequence", None, string_bound)


def _strip_comment(line: str) -> str:
    """Strip a ``#`` comment, keeping ``#`` characters inside quoted string defaults."""
    quote = ""
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = ""
        elif ch in "\"'":
            quote = ch
        elif ch == "#":
            return line[:i]
    return line


def parse_msg(name: str, text: str) -> MessageDefinition:
    """Parse one ``.msg`` text for the type ``name`` (any accepted spelling)."""
    full = normalize_type(name)
    package = _package_of(full)
    fields: list[Field] = []
    constants: dict[str, str] = {}
    for number, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw).strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            raise MsgDefError(f"{full} line {number}: expected '<type> <name>': {raw!r}")
        type_token, rest = parts
        constant = _CONST_RE.match(rest)
        if constant is not None:
            constants[constant.group(1)] = constant.group(2).strip()
            continue
        field_name = rest.split(None, 1)[0]
        if not _NAME_RE.match(field_name):
            raise MsgDefError(f"{full} line {number}: bad field name {field_name!r}")
        try:
            ftype = _parse_type(type_token, package)
        except MsgDefError as exc:
            raise MsgDefError(f"{full} line {number}: {exc}") from exc
        fields.append(Field(field_name, ftype))
    if not fields:
        # rosidl gives empty messages one placeholder byte so that they can be serialized.
        fields.append(Field(EMPTY_PLACEHOLDER, FieldType("uint8")))
    return MessageDefinition(full, fields, constants)


def split_bundle(name: str, text: str) -> list[tuple[str, str]]:
    """Split a ``ros2msg`` schema into ``(type name, .msg text)`` pairs."""
    result: list[tuple[str, str]] = []
    current_name = name
    current: list[str] = []
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.strip() == SEPARATOR:
            result.append((current_name, "\n".join(current)))
            if i + 1 >= len(lines) or not lines[i + 1].startswith("MSG: "):
                if i + 1 < len(lines) and lines[i + 1].startswith("IDL: "):
                    raise MsgDefError(f"{name}: IDL definitions are not supported")
                raise MsgDefError(f"{name}: separator not followed by 'MSG: <type>'")
            current_name = lines[i + 1][5:].strip()
            current = []
            i += 2
            continue
        current.append(line)
        i += 1
    result.append((current_name, "\n".join(current)))
    return result


class Registry:
    """Message definitions by normalised type name."""

    def __init__(self, include_builtins: bool = True) -> None:
        self._defs: dict[str, MessageDefinition] = {}
        if include_builtins:
            for type_name, text in BUILTIN_DEFINITIONS.items():
                self.add(type_name, text)

    def add(self, name: str, text: str) -> MessageDefinition:
        definition = parse_msg(name, text)
        self._defs[definition.name] = definition
        return definition

    def add_bundle(self, name: str, text: str) -> None:
        """Add a ``ros2msg`` schema; its definitions replace built-in ones of the same name."""
        for type_name, body in split_bundle(name, text):
            self.add(type_name, body)

    def __contains__(self, name: object) -> bool:
        if not isinstance(name, str):
            return False
        try:
            return normalize_type(name) in self._defs
        except MsgDefError:
            return False

    def get(self, name: str) -> MessageDefinition:
        full = normalize_type(name)
        try:
            return self._defs[full]
        except KeyError:
            raise MsgDefError(f"no definition for {full}") from None

    def missing_dependencies(self, name: str) -> list[str]:
        """Return the types reachable from ``name`` that have no definition."""
        missing: list[str] = []
        seen: set[str] = set()
        stack = [normalize_type(name)]
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            definition = self._defs.get(current)
            if definition is None:
                missing.append(current)
                continue
            stack.extend(f.type.base for f in definition.fields if not f.type.is_primitive)
        return sorted(missing)

    def names(self) -> list[str]:
        return sorted(self._defs)


# Field lists written from the public ROS 2 interface documentation (common_interfaces,
# rcl_interfaces and geometry2). Comments are omitted; constants that matter are kept.
BUILTIN_DEFINITIONS: dict[str, str] = {
    "builtin_interfaces/msg/Time": "int32 sec\nuint32 nanosec\n",
    "builtin_interfaces/msg/Duration": "int32 sec\nuint32 nanosec\n",
    "std_msgs/msg/Header": "builtin_interfaces/Time stamp\nstring frame_id\n",
    "std_msgs/msg/String": "string data\n",
    "geometry_msgs/msg/Vector3": "float64 x\nfloat64 y\nfloat64 z\n",
    "geometry_msgs/msg/Point": "float64 x\nfloat64 y\nfloat64 z\n",
    "geometry_msgs/msg/Quaternion": ("float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n"),
    "geometry_msgs/msg/Pose": "Point position\nQuaternion orientation\n",
    "geometry_msgs/msg/PoseStamped": "std_msgs/Header header\nPose pose\n",
    "geometry_msgs/msg/PoseWithCovariance": "Pose pose\nfloat64[36] covariance\n",
    "geometry_msgs/msg/Twist": "Vector3 linear\nVector3 angular\n",
    "geometry_msgs/msg/TwistStamped": "std_msgs/Header header\nTwist twist\n",
    "geometry_msgs/msg/TwistWithCovariance": "Twist twist\nfloat64[36] covariance\n",
    "geometry_msgs/msg/Transform": "Vector3 translation\nQuaternion rotation\n",
    "geometry_msgs/msg/TransformStamped": (
        "std_msgs/Header header\nstring child_frame_id\nTransform transform\n"
    ),
    "sensor_msgs/msg/LaserScan": (
        "std_msgs/Header header\n"
        "float32 angle_min\nfloat32 angle_max\nfloat32 angle_increment\n"
        "float32 time_increment\nfloat32 scan_time\n"
        "float32 range_min\nfloat32 range_max\n"
        "float32[] ranges\nfloat32[] intensities\n"
    ),
    "sensor_msgs/msg/Imu": (
        "std_msgs/Header header\n"
        "geometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\n"
        "geometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\n"
        "geometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n"
    ),
    "sensor_msgs/msg/JointState": (
        "std_msgs/Header header\nstring[] name\nfloat64[] position\n"
        "float64[] velocity\nfloat64[] effort\n"
    ),
    "sensor_msgs/msg/Image": (
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\n"
        "uint8 is_bigendian\nuint32 step\nuint8[] data\n"
    ),
    "sensor_msgs/msg/CompressedImage": "std_msgs/Header header\nstring format\nuint8[] data\n",
    "nav_msgs/msg/Odometry": (
        "std_msgs/Header header\nstring child_frame_id\n"
        "geometry_msgs/PoseWithCovariance pose\ngeometry_msgs/TwistWithCovariance twist\n"
    ),
    "tf2_msgs/msg/TFMessage": "geometry_msgs/TransformStamped[] transforms\n",
}
