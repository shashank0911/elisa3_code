import configuration as config
import numpy as np

if __name__ == "__main__":
    print("Hello world")
    ref_lines_domain = np.array([
        [config.domain[0], config.domain[1]],
        [config.domain[1], config.domain[2]],
        [config.domain[2], config.domain[3]],
        [config.domain[3], config.domain[0]]
    ])

    # print ("ref_lines domain: ", ref_lines_domain)
    print ("ref_lines domain type: ", ref_lines_domain.shape)

