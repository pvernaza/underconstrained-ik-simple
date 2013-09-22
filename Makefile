EIGEN_INCLUDE=-I/opt/local/include/eigen3

all: UnderconstrainedIKTest

%.o: %.cc
	$(CXX) $(EIGEN_INCLUDE) -c $< -o $@ 

UnderconstrainedIKTest: UnderconstrainedIKTest.o
	$(CXX) $^ -o $@
