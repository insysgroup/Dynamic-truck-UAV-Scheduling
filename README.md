# Dynamic Truck-UAV Collaboration and Integrated Route Planning for Resilient Urban Emergency Response
## Background
To realize the integrated route planning based on a dynamic truck-UAV collaboration strategy, we proposed a Tabu Search-based Integrated (TSI) scheduling algorithm. This project is developed for the implementation of the TSI scheduling algorithm and the experiments of the paper "_Dynamic Truck-UAV Collaboration and Integrated Route Planning for Resilient Urban Emergency Response_", doi: 10.1109/TEM.2023.3299693.

## Project Structure
-----------------

| Name                               | Description                                                  |
| ---------------------------------- | ------------------------------------------------------------ |
| **.idea**                          | Default files for project configurations.  |
| **env**/**data**             | Datasets used in experiments.              |
| **env**/configuration.py         | Parameter settings.                                 |
| **env**/read_Data.py            | Functions for reading data from **env**/**data**.                            |
| **env**/route_map.py            | Functions for constructing the scenarios.                      |
| **env**/route_map_0.py            | Functions for constructing the scenarios with the road disruptions.|
| **env**/solution.py                        | Solution functions for the DTU collaboration strategy.        |
| **env**/solution_fstp.py                        | Solution functions for the TUFS strategy.        |
| **env**/solution_ptu.py                        | Solution functions for the PTU strategy.        |
| fstsp.py       | TS-based solution algorithm for the TUFS strategy.                                |
| independent.py       | TS-based solution algorithm for the PTU strategy.                                |
| tabu_search.py       | TS-based integrated algorithm for the DTU strategy.                                |
| tabu_search_pure.py       | TS-based solution algorithm for the DTU strategy.        |
| simulated_annealing.py       | SA-based solution algorithm for the TUFS strategy.        |
| variable_neighborhood_search.py       | VNS-based solution algorithm for the TUFS strategy.        |

## Citation
This paper has been accepted by IEEE Transactions on Engineering Management. Feel free to cite us if you found this repository interesting:
```bibtex
@article{long2023dynamic,
    title={Dynamic Truck-UAV Collaboration and Integrated Route Planning for Resilient Urban Emergency Response},
    author={Yuying, Long and Gangyan, Xu and Jinqiu, Zhao and Binglei, Xie and Meng, Fang},
    year={2023},
    month={July},
    doi={10.1109/TEM.2023.3299693}
}
```
