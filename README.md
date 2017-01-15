# RussianAICup-2016 agent

[RussianAICup](http://russianaicup.ru) is russian annual AI contest.
I used deep reinforcement learning algorithm [DDPG](https://arxiv.org/abs/1509.02971). Agent written in C++ and divided into 2 parts. WizardTrainer (depends on TensorFlow) that is used as server for hosting neural network and for training this NN. And agent for participating the contest. So multiple agents can connect and operate simultaneously.
- TensorflowGraph - contains python script to generate neural network
- WizardTrainer - contains WizardTrainer and StateReader (used to translate saved states into .h files to use separately from tensorflow)
- cpp-cgdk - contest agent code
- send - last sended to contest agent
- sends - multiple last sends
- tensor_values - folder for .h files for StateReader

Code is not clear and it is not simple to build programs. So ask me if needed :)

See how my agent performs [here](http://russianaicup.ru/profile/Parilo2) and on the video

[![RussianAICup-2016 Agent](https://img.youtube.com/vi/vM_2Ned9gmY/0.jpg)](https://www.youtube.com/watch?v=vM_2Ned9gmY)
