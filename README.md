# PixarLamp

Design of a physical lamp that accurately matches the famous animation in Pixar's Luxo Jr. Simulation done in Drake, CAD in Fusion 360, and crying in banana lounge.

The associated Deepnote project can be viewed here:
![https://deepnote.com/project/PixarLamp-X9jh03HCR9SHHH7tTis4fA/%2FPixarLamp%2Facrobot.ipynb](https://deepnote.com/project/PixarLamp-X9jh03HCR9SHHH7tTis4fA/%2FPixarLamp%2Facrobot.ipynb).

## Version Control
Version control with this particular setup is a little strange. Deepnote is pulling a docker image stolen from 6.843 with Drake preinstalled, but it doesn't include `git` since it's not designed for active development. This is fixable by running `apt update && apt install git` in a Deepnote terminal, and then version control should work.

Since the GitHub integration is already setup, it should load your GitHub account details automatically and you shouldn't need to login.