import numpy as np
import tempfile
import tensorflow as tf

from tf_rl.controller import ContinuousDeepQ
#from tf_rl.simulation import KarpathyGame
from tf_rl import simulate
from tf_rl.models import MLP

#tf.ops.reset_default_graph()
session = tf.Session()

# This little guy will let us run tensorboard
#      tensorboard --logdir [LOG_DIR]
journalist = tf.train.SummaryWriter("/tmp")

observation_size = 394 #234 #394 #714;#394;#3144;
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

a=192
b=128
c=128
d=128
e=128
#f=tf.nn.tanh
#f2=tf.nn.tanh
f=tf.nn.relu
f2=tf.nn.tanh

critic = MLP([input_size, num_actions], [a, b, c, d, e, 1],
            [f, f, f, f, f2, tf.identity], scope='critic')

actor = MLP([input_size,], [a, b, c, d, e, num_actions],
            [f, f, f, f, f2, tf.identity], scope='actor')

# The optimizer to use. Here we use RMSProp as recommended
# by the publication
#optimizer = tf.train.RMSPropOptimizer(learning_rate= 0.0001, decay=0.9)
#optimizer = tf.train.RMSPropOptimizer(learning_rate= 0.0005, decay=0.9)
optimizer = tf.train.AdamOptimizer(learning_rate= 0.00001)#, epsilon=0.001)
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
for variable in tf.trainable_variables():
    tf.identity (variable, name="readVariable")
    tf.assign (variable, tf.placeholder(tf.float32, variable.get_shape(), name="variableValue"), name="resoreVariable")

tf.train.write_graph(session.graph_def, 'models/', 'wt-graph-2d-ddpg.pb', as_text=False)
