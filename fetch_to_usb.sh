#!/bin/bash
#
# fetch_to_usb.sh - Mount a USB drive on this Pi, then pull capture
# directories from the TDA board over SSH into /mnt/usb/TDA_output,
# skipping anything that already exists locally (safe to re-run).
#
# Usage:
#   ./fetch_to_usb.sh /dev/sda1                              # copy ALL captures (sequential)
#   ./fetch_to_usb.sh /dev/sda1 -p                            # copy ALL captures (parallel per-file, faster)
#   ./fetch_to_usb.sh /dev/sda1 -c capture_1784577483         # copy ONE capture
#   ./fetch_to_usb.sh /dev/sda1 -c capture_1784577483 -p      # copy ONE capture, parallel
#   ./fetch_to_usb.sh /dev/sda1 -c capture_1784577483 /mnt/ssd  # + custom remote base
#
#   $1        = USB device/partition (e.g. /dev/sda1) - check with `lsblk` first
#   -c NAME   = optional: only fetch this one capture directory
#   -p        = optional: transfer files within each directory in parallel
#               (confirmed ~2.6-3x faster on this board's slow single-connection
#               scp - Dropbear/old SoC bottleneck is per-connection, not a hard
#               aggregate cap). Off by default; original sequential scp -r is
#               simpler and proven, so it stays the default.
#   $2 (or last positional) = optional: remote base dir on TDA, default /mnt/ssd
#
set -euo pipefail

MOUNT_POINT="/mnt/usb"
TDA_HOST="root@192.168.33.180"
SSH_OPTS=(-o HostKeyAlgorithms=+ssh-rsa -o PubkeyAuthentication=no)

# --- parse args: pull out -c/--capture and -p/--parallel first, keep the rest positional ---
SINGLE_CAPTURE=""
PARALLEL=false
POSITIONAL=()
while [ $# -gt 0 ]; do
    case "$1" in
        -c|--capture)
            SINGLE_CAPTURE="${2:?-c/--capture requires a capture directory name}"
            shift 2
            ;;
        -p|--parallel)
            PARALLEL=true
            shift
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done

USB_DEV="${POSITIONAL[0]:?Usage: $0 /dev/sdXX [-c capture_dir_name] [remote_base_dir]}"
REMOTE_BASE="${POSITIONAL[1]:-/mnt/ssd}"

# --- sanity checks before touching anything ---
if [ ! -b "$USB_DEV" ]; then
    echo "ERROR: $USB_DEV is not a block device. Run 'lsblk' to find the right one." >&2
    exit 1
fi

if mountpoint -q "$MOUNT_POINT" 2>/dev/null; then
    echo "ERROR: $MOUNT_POINT is already mounted. Unmount it first (sudo umount $MOUNT_POINT) or pick another mount point." >&2
    exit 1
fi

echo "==> Confirming SSH connectivity to $TDA_HOST ..."
if ! ssh "${SSH_OPTS[@]}" -o ConnectTimeout=5 "$TDA_HOST" 'echo ok' >/dev/null 2>&1; then
    echo "ERROR: Could not reach $TDA_HOST over SSH. Check the board is powered on and network is up." >&2
    exit 1
fi

# --- mount ---
echo "==> Mounting $USB_DEV at $MOUNT_POINT ..."
sudo mkdir -p "$MOUNT_POINT"

FSTYPE="$(sudo blkid -o value -s TYPE "$USB_DEV" 2>/dev/null || true)"
echo "    Detected filesystem: ${FSTYPE:-unknown}"

case "$FSTYPE" in
    vfat|exfat)
        # FAT-family filesystems have no Unix ownership concept - chown
        # always fails with "Operation not permitted", even as root.
        # Set ownership at mount time instead via uid/gid options.
        sudo mount -o "uid=$(id -u),gid=$(id -g)" "$USB_DEV" "$MOUNT_POINT"
        ;;
    ntfs|ntfs3)
        sudo mount -t ntfs3 -o "uid=$(id -u),gid=$(id -g)" "$USB_DEV" "$MOUNT_POINT" \
            || sudo mount -o "uid=$(id -u),gid=$(id -g)" "$USB_DEV" "$MOUNT_POINT"
        ;;
    *)
        # Native Linux filesystem (ext4, etc.) - normal mount + chown works
        sudo mount "$USB_DEV" "$MOUNT_POINT"
        sudo chown "$(whoami)" "$MOUNT_POINT"
        ;;
esac

# NOTE: if this drive is FAT32, files over 4GB will fail to copy - your
# capture .bin files can easily exceed that. Reformat as exFAT or ext4 if
# you hit "File too large" errors:
#   sudo mkfs.exfat "$USB_DEV"   (needs exfat-utils/exfatprogs installed)

OUTPUT_DIR="$MOUNT_POINT/TDA_output"
mkdir -p "$OUTPUT_DIR"

# --- determine which directories to fetch ---
if [ -n "$SINGLE_CAPTURE" ]; then
    echo "==> Single-capture mode: checking $REMOTE_BASE/$SINGLE_CAPTURE exists on the TDA board ..."
    if ! ssh "${SSH_OPTS[@]}" "$TDA_HOST" "test -d '$REMOTE_BASE/$SINGLE_CAPTURE'"; then
        echo "ERROR: $REMOTE_BASE/$SINGLE_CAPTURE does not exist on the TDA board." >&2
        sudo umount "$MOUNT_POINT"
        exit 1
    fi
    REMOTE_DIRS=("$SINGLE_CAPTURE")
else
    echo "==> Listing capture directories under $REMOTE_BASE on the TDA board ..."
    # NOTE: the TDA board runs BusyBox find, which doesn't support -printf -
    # get full paths back instead and strip to basenames locally.
    mapfile -t REMOTE_PATHS < <(
        ssh "${SSH_OPTS[@]}" "$TDA_HOST" \
            "find '$REMOTE_BASE' -mindepth 1 -maxdepth 1 -type d"
    )
    REMOTE_DIRS=()
    for p in "${REMOTE_PATHS[@]}"; do
        REMOTE_DIRS+=("$(basename "$p")")
    done
    if [ "${#REMOTE_DIRS[@]}" -eq 0 ]; then
        echo "No capture directories found under $REMOTE_BASE. Nothing to do."
        sudo umount "$MOUNT_POINT"
        exit 0
    fi
    echo "Found ${#REMOTE_DIRS[@]} directories on the TDA board."
fi

# --- copy each one, skipping anything already present locally ---
COPIED=0
SKIPPED=0
for dir in "${REMOTE_DIRS[@]}"; do
    DEST="$OUTPUT_DIR/$dir"
    if [ -d "$DEST" ]; then
        echo "==> SKIP  $dir (already exists at $DEST)"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    if [ "$PARALLEL" = true ]; then
        echo "==> COPY  $dir (parallel per-file transfer) ..."
        mkdir -p "$DEST"

        # List remote files in this directory (BusyBox find - no -printf, so
        # get full paths and strip locally, same approach as the directory listing above)
        mapfile -t REMOTE_FILE_PATHS < <(
            ssh "${SSH_OPTS[@]}" "$TDA_HOST" \
                "find '$REMOTE_BASE/$dir' -mindepth 1 -maxdepth 1 -type f"
        )

        if [ "${#REMOTE_FILE_PATHS[@]}" -eq 0 ]; then
            echo "    WARNING: no files found in $dir - skipping" >&2
            rmdir "$DEST" 2>/dev/null || true
            continue
        fi

        # Fire all files in this directory as parallel scp jobs, then wait for
        # all of them - confirmed ~2.6-3x aggregate throughput vs sequential
        # single-connection transfer on this board (Dropbear/old SoC bottleneck
        # is per-connection, not a hard aggregate cap).
        PIDS=()
        for fpath in "${REMOTE_FILE_PATHS[@]}"; do
            fname="$(basename "$fpath")"
            scp -O "${SSH_OPTS[@]}" "$TDA_HOST:$fpath" "$DEST/$fname" &
            PIDS+=($!)
        done

        TRANSFER_OK=true
        for pid in "${PIDS[@]}"; do
            if ! wait "$pid"; then
                TRANSFER_OK=false
            fi
        done

        if [ "$TRANSFER_OK" = true ]; then
            COPIED=$((COPIED + 1))
        else
            echo "    WARNING: one or more files in $dir failed to transfer - removing partial copy" >&2
            rm -rf "$DEST"
        fi
    else
        echo "==> COPY  $dir ..."
        if scp -O -r "${SSH_OPTS[@]}" "$TDA_HOST:$REMOTE_BASE/$dir" "$OUTPUT_DIR/"; then
            COPIED=$((COPIED + 1))
        else
            echo "    WARNING: transfer of $dir failed or was incomplete - removing partial copy" >&2
            rm -rf "$DEST"
        fi
    fi
done

echo ""
echo "==> Summary: copied $COPIED, skipped $SKIPPED (already present)"

echo "==> Syncing filesystem buffers to disk before unmount ..."
sync

echo "==> Unmounting $MOUNT_POINT ..."
sudo umount "$MOUNT_POINT"

echo "Done. Safe to remove the USB drive."
