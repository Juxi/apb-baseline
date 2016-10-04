For a new computer:
* `mkdir ~/.ssh`
* `cd ~/.ssh/`
* `chmod 700 ~/.ssh`
* `ssh-keygen -t rsa`
* Enter `apc_id_rsa`
* No pass phrase
* `ssh -oHostKeyAlgorithms='ssh-rsa' dockpc@dockpc-nuc`
* Say YES
* `exit`
* `ssh-copy-id dockpc@dockpc-nuc`
