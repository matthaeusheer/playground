from path_planning.rrt import rrt_path_finder
from path_planning.config import RrtConfig
from path_planning.rrt import SimulationState

if __name__ == '__main__':
    cfg = RrtConfig()
    simulation_state = SimulationState(cfg)
    rrt_path_finder(cfg, simulation_state)
