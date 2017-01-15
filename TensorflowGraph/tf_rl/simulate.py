import time

#from IPython.display import clear_output, display, HTML
#from os.path import join, exists
#from os import makedirs

#from tf_rl.utils.event_queue import EventQueue

def simulate(simulation,
             controller = None,
             fps=60,
             actions_per_simulation_second=60,
             simulation_resultion=0.001,
             speed=1.0,
             disable_training=False,
             save_path=None):
    """Start the simulation. Performs three tasks

        - visualizes simulation in iPython notebook
        - advances simulator state
        - reports state to controller and chooses actions
          to be performed.

    Parameters
    -------
    simulation: tr_lr.simulation
        simulation that will be simulated ;-)
    controller: tr_lr.controller
        controller used
    fps: int
        frames per seconds to display;
        decrease for faster training.
    actions_per_simulation_second: int
        how many times perform_action is called per
        one second of simulation time
    speed: float
        executed <speed> seconds of simulation time
        per every second of real time
    disable_training: bool
        if true training_step is never called.
    save_path: str
        save svg visualization (only tl_rl.utils.svg
        supported for the moment)
    """
#    eq = EventQueue()
#
#    time_between_frames  = 1.0 / fps
#    simulation_time_between_actions = 1.0 / actions_per_simulation_second
#
#    simulation_resultion /= speed
#
#    vis_s = {
#        'last_image': 0
#    }
#
#    if save_path is not None:
#        if not exists(save_path):
#            makedirs(save_path)
#
#    ###### VISUALIZATION
#    def visualize():
#        clear_output(wait=True)
#        svg_html = simulation.to_html()
#        display(svg_html)
#        if save_path is not None:
#            img_path = join(save_path, "%d.svg" % (vis_s['last_image'],))
#            with open(img_path, "w") as f:
#                svg_html.write_svg(f)
#            vis_s['last_image'] += 1
#
#    eq.schedule_recurring(visualize, time_between_frames)
#
#
#    ###### CONTROL
#    ctrl_s = {
#        'last_observation': None,
#        'last_action':      None,
#    }
#
#    def control():
#        # sense
#        new_observation = simulation.observe()
#        reward          = simulation.collect_reward()
#        # store last transition
#        if ctrl_s['last_observation'] is not None:
#            controller.store(ctrl_s['last_observation'], ctrl_s['last_action'], reward, new_observation)
#
#        # act
#        new_action = controller.action(new_observation)
#        simulation.perform_action(new_action)
#
#        #train
#        if not disable_training:
#            controller.training_step()
#
#        # update current state as last state.
#        ctrl_s['last_action'] = new_action
#        ctrl_s['last_observation'] = new_observation
#
#
#
#    ##### SIMULATION
#    sim_s = {
#        'simulated_up_to':             time.time(),
#        'simulation_time_since_last_action': 0,
#    }
#    def advance_simulation():
#        while sim_s['simulated_up_to'] < time.time():
#            simulation.step(simulation_resultion)
#            sim_s['simulated_up_to'] += simulation_resultion / speed
#            sim_s['simulation_time_since_last_action'] += simulation_resultion
#            if sim_s['simulation_time_since_last_action'] > simulation_time_between_actions:
#                if controller is not None:
#                    control()
#                    sim_s['simulation_time_since_last_action'] = 0
#
#    eq.schedule_recurring(advance_simulation, time_between_frames)
#
#    eq.run()
