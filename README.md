# Setting up a new SSD

1. Download the `new_ssd_setup.sh` scrip (in the scripts folder).
2. Make it executable (`chmod +x new_ssd_setup.sh`).
3. Run it.

It's mostly automatic - you will be prompted to add the SSH key to GitHub and to download and test the latest Zed SDK. It will also prompt for reboots


## Alternatively: Setting up the startup scripts (manually)
These scripts assume that the user is `bristol-fsai`. Change this across all scripts if this isn't the case.
This only needs to be executed once on the SSD. Just copy the `rc.local` script to `/etc/rc.local` (with root perms): 

```bash
sudo cp launch/rc.local /etc/rc.local
sudo chmod +x /etc/rc.local # make it executable
```

If you ever want to disable, the scripts, rename the `root_launch` to `root_launch_disabled` (and rename back to re-enable).
