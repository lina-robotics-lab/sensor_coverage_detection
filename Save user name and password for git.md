## Useful note: set user name and password for git remote repository

You can use the git config to enable credentials storage in git.

'''
git config --global credential.helper store
'''

When running this command, the first time you pull or push from the remote repository, you'll get asked about the username and password.

Afterwards, for consequent communications with the remote repository you don't have to provide the username and password.

