"""
os module for RT1021-MicroPython v2.2.0
Filesystem and system operations.
"""

from typing import Any, List, Optional, Tuple


def chdir(path: str) -> None:
    """
    Change current working directory.

    :param path: Target directory path
    """
    ...

def getcwd() -> str:
    """Return current working directory path."""
    ...

def listdir(dir: str = ".") -> List[str]:
    """
    List directory contents.

    :param dir: Directory to list (defaults to current)
    :return: List of file/directory names
    """
    ...

def mkdir(path: str) -> None:
    """
    Create a directory.

    :param path: Directory path to create
    """
    ...

def remove(path: str) -> None:
    """
    Delete a file.

    :param path: File path to delete
    """
    ...

def rmdir(path: str) -> None:
    """
    Delete an empty directory.

    :param path: Directory path to delete
    """
    ...

def rename(old_path: str, new_path: str) -> None:
    """
    Rename a file or directory.

    :param old_path: Current path
    :param new_path: New path
    """
    ...

def stat(path: str) -> Tuple[int, int, int, int, int, int, int, int, int, int]:
    """
    Get file status.

    :param path: File path
    :return: 10-tuple (mode, ino, dev, nlink, uid, gid, size, atime, mtime, ctime)
    """
    ...

def statvfs(path: str) -> Tuple[int, int, int, int, int, int, int, int, int, int]:
    """
    Get filesystem status.

    :param path: Path on the filesystem
    :return: 10-tuple (bsize, frsize, blocks, bfree, bavail, files, ffree, favail, flag, namemax)
    """
    ...

def sync() -> None:
    """Sync all filesystems."""
    ...

def urandom(n: int) -> bytes:
    """
    Return n random bytes from hardware RNG.

    :param n: Number of bytes
    :return: Random bytes
    """
    ...

def dupterm(stream_object: Any, index: int = 0) -> Any:
    """
    Duplicate or switch UART to stdin/stdout.

    :param stream_object: Stream-like object or None to disable
    :param index: Terminal index
    """
    ...

def mount(fsobj: Any, mount_point: str, *, readonly: bool = False) -> None:
    """
    Mount a filesystem.

    :param fsobj: Filesystem object (e.g. VfsFat)
    :param mount_point: Mount point path
    :param readonly: Mount as read-only
    """
    ...

def umount(mount_point: str) -> None:
    """
    Unmount a filesystem.

    :param mount_point: Mount point path
    """
    ...

def uname() -> Tuple[str, str, str, str, str]:
    """
    Return system information.

    :return: (sysname, nodename, release, version, machine)
    """
    ...
