---
name: Bug report
about: A bag that is read wrongly, a wrong finding, or a crash
labels: bug
---

**What happened**

**What you expected**

**Command and output**

```console
rosbag-inspect check ... --json
```

**About the bag**

- Recorded with (ROS 2 distribution, `ros2 bag record` options, storage plugin):
- `rosbag-inspect info BAG --json` output:
- A few decoded messages of the topic involved (`rosbag-inspect echo BAG TOPIC --limit 3`), if
  relevant. Please do not attach bags you are not allowed to share.

**Environment**

- rosbag-inspect version (`rosbag-inspect --version`):
- Python version and OS:
