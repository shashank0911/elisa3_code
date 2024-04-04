import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

data_exec_cpp_two = pd.read_csv('~/catkin_ws/src/estimator/data/profiler_2_bots.csv', header=None)
data_loop_cpp_two = pd.read_csv('~/catkin_ws/src/estimator/data/profiler_loop_2_bots.csv', header=None)
data_exec_cpp_four = pd.read_csv('~/catkin_ws/src/estimator/data/profiler_4_bots_fin.csv', header=None)
data_loop_cpp_four = pd.read_csv('~/catkin_ws/src/estimator/data/profiler_loop_4_bots_fin.csv', header=None)
data_loop_py_four = pd.read_csv('~/alternate_catkin_ws/src/controller_py_v2/src/data/loop_time_4_bots.csv', header=None)

data_exec_cpp_two[1] = data_exec_cpp_two[1] * 1000
data_exec_cpp_two[1] = data_exec_cpp_two[1].round(2)
avg_exec_cpp_two = data_exec_cpp_two[1].mean()
print(avg_exec_cpp_two)

data_exec_cpp_four[1] = data_exec_cpp_four[1] * 1000
data_exec_cpp_four[1] = data_exec_cpp_four[1].round(2)
avg_exec_cpp_four = data_exec_cpp_four[1].mean()
print(avg_exec_cpp_four)

data_loop_cpp_two[1] = data_loop_cpp_two[1] * 1000
data_loop_cpp_two[1] = data_loop_cpp_two[1].round(2)
avg_loop_cpp_two = data_loop_cpp_two[1].mean()

data_loop_cpp_four[1] = data_loop_cpp_four[1] * 1000
data_loop_cpp_four[1] = data_loop_cpp_four[1].round(2)
avg_loop_cpp_four = data_loop_cpp_four[1].mean()


data_loop_py_four = data_loop_py_four.iloc[1:]
data_loop_py_four[0] = data_loop_py_four[0].astype('int64')
data_loop_py_four[1] = data_loop_py_four[1].astype('float64')
data_loop_py_four[1] = data_loop_py_four[1] * 1000
data_loop_py_four[1] = data_loop_py_four[1].round(2)
avg_loop_py_four = data_loop_py_four[1].mean()
print(avg_loop_py_four)
dp = data_loop_py_four.shape[0]
# print(data_exec_cpp_four.iloc[:dp])

plt.bar(range(dp), data_loop_py_four[1].iloc[:], color='orange', label='Execution time for python estimator node')
plt.bar(range(dp), data_exec_cpp_four[1].iloc[:dp], color='blue', label='Execution time for cpp estimator node')
plt.axhline(y=avg_loop_py_four, color='red', linestyle='--', label='Average execution time for python estimator node')
plt.axhline(y=avg_exec_cpp_four, color='green', linestyle='--', label='Average execution time for cpp estimator node')
plt.xlabel('Loop instance')
plt.ylabel('Execution time (in msec)')
plt.title('Comparison of loop execution time for python code and cpp code')

def format_ticks(x, pos):
    return '{:.0f}'.format(x)

plt.gca().yaxis.set_major_formatter(FuncFormatter(format_ticks))

plt.legend()

plt.show()


# print(data_loop_cpp_four.iloc[:dp])
plt.bar(range(dp), data_loop_py_four[1], color='orange', label='Execution time for python estimator node')
plt.bar(range(dp), data_loop_cpp_four[1].iloc[1:dp+1], color='blue', label='Loop time for cpp estimator node')
plt.axhline(y=avg_loop_py_four, color='red', linestyle='--', label='Average execution time for python estimator node')
plt.axhline(y=avg_loop_cpp_four, color='green', linestyle='--', label='Average execution time for cpp estimator node')
plt.xlabel('Loop instance')
plt.ylabel('Execution time (in msec)')
plt.title('Comparison of loop execution time for python code and cpp code')

def format_ticks(x, pos):
    return '{:.0f}'.format(x)

plt.gca().yaxis.set_major_formatter(FuncFormatter(format_ticks))

plt.legend()

plt.show()



plt.bar(range(dp*2), data_exec_cpp_four[1].iloc[:dp*2], color='orange', label='Execution time for 4 robots')
plt.bar(range(dp*2), data_exec_cpp_two[1].iloc[:dp*2], color='blue', label='Execution time for 2 robots')
plt.axhline(y=avg_exec_cpp_two, color='green', linestyle='--', label='Average execution time for 2 robots')
plt.axhline(y=avg_exec_cpp_four, color='red', linestyle='--', label='Average execution time for 4 robots')
plt.xlabel('Loop instance')
plt.ylabel('Execution time (in msec)')
plt.title('Comparison of loop execution time for cpp code in case of 2 and 4 robots')

def format_ticks(x, pos):
    return '{:.2f}'.format(x)

plt.gca().yaxis.set_major_formatter(FuncFormatter(format_ticks))

plt.legend()

plt.show()
