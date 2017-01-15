import numpy as np
import tempfile
import tensorflow as tf

from tf_rl.controller import ContinuousDeepQ
#from tf_rl.simulation import KarpathyGame
from tf_rl import simulate
from tf_rl.models import MLP
from tf_rl.conv_model import CNN

#tf.ops.reset_default_graph()
session = tf.Session()

# This little guy will let us run tensorboard
#      tensorboard --logdir [LOG_DIR]
journalist = tf.train.SummaryWriter("/tmp")

observation_size = 714 #1280+74 #714;#394;#3144;
observations_in_seq = 1;
input_size = observation_size*observations_in_seq;

# actions
num_actions = 6;

#brain = MLP([input_size,], [5, 5, 5, num_actions], 
#            [tf.tanh, tf.tanh, tf.tanh, tf.identity])
#brain = MLP([input_size,], [20, 20, 20, 20, num_actions], 
#            [tf.tanh, tf.tanh, tf.tanh, tf.tanh, tf.identity])

#brain = MLP([input_size,], [32, 32, 32, 32, 32, num_actions], 
#            [tf.nn.relu, tf.nn.relu, tf.nn.relu, tf.nn.relu, tf.nn.relu, tf.identity])


cnn_input = [1, 640] #[1, 1280]
mlp_addition_actor = 74
mlp_addition_critic = 74 + num_actions

#cnn_layers =  [[1, 5, 1, 6], [1, 5, 6, 16], [1, 4, 16, 24]] #conv size x, conv size y, in channels, out channels
#cnn_strides = [[1, 1, 5, 1], [1, 1, 1,  1], [1, 1,  1,  1]] #1, stride x, stride y, 1
#cnn_pooling = [[1, 2], [1, 4], [1, 2]]
#336

cnn_layers =  [[1, 5, 1, 6], [1, 5, 6, 16], [1, 5, 16, 24]] #conv size x, conv size y, in channels, out channels
cnn_strides = [[1, 1, 5, 1], [1, 1, 1,  1], [1, 1,  1,  1]] #1, stride x, stride y, 1
cnn_pooling = [[1, 2], [1, 2], [1, 2]]
#312

#cnn_layers =  [[1, 5, 1, 3], [1, 5, 3, 6], [1, 5, 6, 12]] #conv size x, conv size y, in channels, out channels
#cnn_strides = [[1, 1, 5, 1], [1, 1, 1,  1], [1, 1,  1,  1]] #1, stride x, stride y, 1
#cnn_pooling = [[1, 2], [1, 2], [1, 2]]
#348

cnn_nonlinearities = [tf.nn.relu, tf.nn.relu, tf.nn.relu]

a=512
b=512
c=256
#d=512
#e=256
f=tf.nn.relu
f2=tf.nn.tanh

mlp_input_actor = 312 + mlp_addition_actor
mlp_input_critic = 312 + mlp_addition_critic
mlp_hiddens_actor  = [a, b, c, c, c, c, c, c, num_actions]
mlp_hiddens_critic = [a, b, c, c, c, c, c, c, 1]
mlp_nonlinearities = [f, f, f, f, f, f, f, f2, tf.identity]

#cnn_net = CNN (cnn_input, mlp_addition, cnn_layers, cnn_strides, cnn_pooling, cnn_nonlinearities, mlp_input, mlp_hiddens, mlp_nonlinearities, "test")
#cnn_copy = cnn_net.copy ("test_copy")

critic = CNN (cnn_input, mlp_addition_critic, cnn_layers, cnn_strides, cnn_pooling, cnn_nonlinearities, mlp_input_critic,  mlp_hiddens_critic, mlp_nonlinearities, "critic")
actor = CNN  (cnn_input, mlp_addition_actor,  cnn_layers, cnn_strides, cnn_pooling, cnn_nonlinearities, mlp_input_actor,   mlp_hiddens_actor,  mlp_nonlinearities, "actor")

#critic = MLP([input_size, num_actions], [a, b, c, d, e, 1],
#            [f, f, f, f2, f2, tf.identity], scope='critic')

#actor = MLP([input_size,], [a, b, c, d, e, num_actions],
#            [f, f, f, f2, f2, tf.identity], scope='actor')

# The optimizer to use. Here we use RMSProp as recommended
# by the publication
#optimizer = tf.train.RMSPropOptimizer(learning_rate= 0.0001, decay=0.9)
#optimizer = tf.train.RMSPropOptimizer(learning_rate= 0.0005, decay=0.9)
optimizer = tf.train.AdamOptimizer(learning_rate= 0.000002)
#optimizer = tf.train.GradientDescentOptimizer(learning_rate= 0.001)

# DiscreteDeepQ object
current_controller = ContinuousDeepQ(input_size, num_actions, actor, critic, optimizer, session, discount_rate=0.99, target_actor_update_rate=0.01, target_critic_update_rate=0.01, exploration_period=5000, max_experience=10000, store_every_nth=4, train_every_nth=4, summary_writer=journalist)

#class ContinuousDeepQ
#                       observation_size,
#                       action_size,
#                       actor,
#                       critic,
#                       optimizer,
#                       session,
#                       exploration_sigma=0.05,
#                       exploration_period=1000,
#                       store_every_nth=5,
#                       train_every_nth=5,
#                       minibatch_size=32,
#                       discount_rate=0.95,
#                       max_experience=30000,
#                       target_actor_update_rate=0.01,
#                       target_critic_update_rate=0.01,
#                       summary_writer=None



init_all_vars_op = tf.initialize_variables(tf.all_variables(), name='init_all_vars_op')

session.run(tf.initialize_all_variables())

#for saving graph state, trainable variable values
print ("--- variables")
for variable in tf.trainable_variables():
    rv = tf.identity (variable, name="readVariable")
    print (rv.name + " " + variable.name + " shape: " + str(rv.get_shape ()))
    tf.assign (variable, tf.placeholder(tf.float32, variable.get_shape(), name="variableValue"), name="resoreVariable")

tf.train.write_graph(session.graph_def, 'models/', 'wt-graph-2d-ddpg-cnn.pb', as_text=False)
