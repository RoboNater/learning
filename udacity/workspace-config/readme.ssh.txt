Notes:
ssh configured 2025-02-22
placed in /workspace/.ssh
Github /workspace/.ssh/id_rsa_github and id_rsa_github.pub
Github passphrase detected
config says to use above file for github.com

# Need to run following each time container rebooted
cp -pr /workspace/.ssh /root/.ssh
eval "$(ssh-agent -s)"
ssh-add /root/.ssh/id_rsa_github

ssh -T git@github.com
# user.email=26860048+RoboNater@users.noreply.github.com
git config --global user.email 26860048+RoboNater@users.noreply.github.com

git config --global user.name RoboNater
