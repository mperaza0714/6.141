# final_challenge

RSS 2021 Final Challenge

Fork this repo and develop your code for the final challenge there!

## Things you need to do

### Initial setup

In order to simplify keeping track of all the various components involved in your system, we will ask you to consolidate the various packages you have been working on in this single repository (using Git submodules -- these instructions will guide you through this process).

Therefore, we recommend you create a blank catkin workspace for the final challenge to ensure you are not creating duplicate packages, and so you can be sure all the various packages of your system are indeed contained within this repo. One possible way to organize it would be as follows:

```bash
mkdir -p ~/challenge_ws/src # create a blank directory to be used as you catkin workspace
cd ~/challenge_ws/src       # go to its src directory
[FOLLOW THE STEPS BELOW]    # Initialize the final challenge repo into this src directory
cd ~/challenge_ws
catkin_make                 # initialize and build the new catkin workspace
```

### Populating this repository with all the required components

First clone the repo.

```bash
git clone git@github.mit.edu:rss/final_challenge.git
cd final_challenge
```

Note that there are several submodules in this repo. They're all currently pointing to labs 3-6 from the public github. You are going to want to change those submodule pointers to use your repos.
To do this, edit the [`.gitmodules`](/.gitmodlues) and replace the url associated with each submodule to the one your team uses. Then, run the following:

```bash
git submodule update --init --recursive
```

You will also have to add your `safety-controller` to this repository! Do this:

```bash
git submodule add git@github.mit.edu:YOURTEAMORG/safety_controller.git
```

Or something similar. You'll be required to run your safety controller during the final challenge.

There are `.gitignores` in every directory in the [`final_challenge`](/final_challenge) directory; you'll want to delete or adapt these when you populate the directories.

(Note: If you didn't fork this repo but instead cloned it, you should update the origin remotes of this repo, as they will be pointing to this template hosted on GitHub. Instead, you will want to update the origin remote to point to your team's own final challenge repo and push.)

## Reminders about submodules

You will almost certainly make changes to your submodules and push them to their respective remotes. To get those updates here, do this:

```bash
cd MODULE
git checkout 'b'
cd ../
```

Where `MODULE` is the directory containing your submodule and `'b'` is the branch, tag, or commit you want to keep in this repo. You'll then have to do:

```bash
git add MODULE
git commit -m 'Some nice message about how you're updating the submodule ref'
git push
```

Note that any changes you make inside the submodule need to be pushed from **within** that submodule.'

If you make any future changes to the url of a submodule, run this:

```bash
git submodule sync
git submodule update --init
```
