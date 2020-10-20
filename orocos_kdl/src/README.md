```sh
emcc --bind -o wasm.js wasm.cpp chain.cpp segment.cpp joint.cpp frames.cpp rigidbodyinertia.cpp rotationalinertia.cpp -I ../../../eigen -s LLD_REPORT_UNDEFINED --no-entry
```

#### TODO
- try: from https://orocos.org/kdl/examples.html#comment-1724

    ```c++
    //Creation of the chain:
    KDL::Chain chain;
    chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
    chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    chain.addSegment(Segment(Joint(Joint::RotZ)));
     
    //Creation of the solvers:
    ChainFkSolverPos_recursive fksolver1(chain1);//Forward position solver
    ChainIkSolverVel_pinv iksolver1v(chain1);//Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(chain1,fksolver1,iksolver1v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
     
    //Creation of jntarrays:
    JntArray q(chain.getNrOfJoints());
    JntArray q_init(chain.getNrOfJoints());
     
    //Set destination frame
    Frame F_dest=...;
     
    int ret = iksolverpos.CartToJnt(q_init,F_dest,q);
  ```

  ```sh
  emcc --bind -o wasm.js wasm.cpp chain.cpp segment.cpp joint.cpp frames.cpp rigidbodyinertia.cpp rotationalinertia.cpp chainfksolverpos_recursive.cpp framevel.cpp frameacc.cpp jntarray.cpp jntarrayvel.cpp jntarrayacc.cpp chainiksolvervel_pinv.cpp chainjnttojacsolver.cpp jacobian.cpp utilities/SVD_HH.cpp chainiksolverpos_lma.cpp utilities/utility.cxx -I ../../../eigen -s LLD_REPORT_UNDEFINED --no-entry && python -m SimpleHTTPServer
  ```
