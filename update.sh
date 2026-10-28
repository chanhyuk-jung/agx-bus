#!/bin/bash
SOURCE_BRANCH="pyAgxArm-source"
STAGING_BRANCH="pyAgxArm-staging"

echo
echo "Fetch new changes from pyAgxArm"
echo ---------------------------------
git remote add pyAgxArm https://github.com/agilexrobotics/pyAgxArm.git

git fetch pyAgxArm master

LATEST_COMMIT=`git ls-remote pyAgxArm | grep "refs/heads/master" | awk '{ print $1}'`
echo
echo "pyAgxArm latest commit: ${LATEST_COMMIT}"
echo

# checkout source repo
git checkout -b ${SOURCE_BRANCH} pyAgxArm/master

# create new staging branch from all the commits impacting "/my-chart" from source repo
git subtree split -P pyAgxArm -b ${STAGING_BRANCH}

# checkout main
git checkout main

# take commits in the staging branch and set "/helm-charts/my-chart" as the commit root
# after you run this script the first time, update the command below to:
# "git subtree merge" with the same parameters
#
echo
echo "Add in changes"
echo ----------------
git subtree merge --squash -P src/agx_bus/pyAgxArm ${STAGING_BRANCH} --message "Update pyAgxArm from https://github.com/agilexrobotics/pyAgxArm/commit/${LATEST_COMMIT}"

# clean up
echo
echo "Removing branches"
echo -------------------
git branch -D ${STAGING_BRANCH}
git branch -D ${SOURCE_BRANCH}

git remote remove pyAgxArm
