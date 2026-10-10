"""Read a rosbag2 directory into plain Python. The one ROS-dependent module.

    read_bag(path, topics) -> {topic: [(t_receive, msg), ...]}

t_receive is the bag's receive time in seconds. Topics absent from the bag map
to empty lists; message types are taken from the bag, so nothing here names a
message package.
"""

import os
from typing import Dict, Iterable, List, Tuple


def stamp_sec(stamp) -> float:
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def _storage_id(path: str) -> str:
    meta = os.path.join(path, 'metadata.yaml')
    if os.path.isfile(meta):
        with open(meta) as f:
            for line in f:
                if 'storage_identifier' in line:
                    return line.split(':', 1)[1].strip()
    if os.path.isfile(path) and path.endswith('.mcap'):
        return 'mcap'
    return 'sqlite3'


def bag_topics(path: str) -> Dict[str, str]:
    """{topic: type} of the bag."""
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('', ''))
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_bag(path: str, topics: Iterable[str]) -> Dict[str, List[Tuple[float, object]]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if not os.path.exists(path):
        raise FileNotFoundError(path)
    wanted = list(topics)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=_storage_id(path)),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in wanted if t in types]
    out: Dict[str, List[Tuple[float, object]]] = {t: [] for t in wanted}
    if not present:
        return out
    reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    classes = {t: get_message(types[t]) for t in present}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, classes[topic])))
    return out


def write_bag(path: str, topic_types: Dict[str, str],
              messages: Iterable[Tuple[str, float, object]]) -> None:
    """Write (topic, t_receive, msg) triples to a new sqlite3 bag; for tests
    and synthetic recordings."""
    import rosbag2_py
    from rclpy.serialization import serialize_message

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, type_ in topic_types.items():
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=type_, serialization_format='cdr'))
    for topic, t, msg in messages:
        writer.write(topic, serialize_message(msg), int(round(t * 1e9)))
    del writer
