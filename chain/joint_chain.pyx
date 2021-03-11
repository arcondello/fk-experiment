# distutils: language = c++
# cython: language_level=3

import numpy as np


cdef extern from "chain.h" namespace "chain" nogil:
    cdef cppclass cppJointChain "chain::JointChain" [Scalar]:
        cppJointChain()
        cppJoint[Scalar]& add_joint(const Scalar*, const Scalar*)
        cppJoint[Scalar]& base()
        cppJoint[Scalar]& get_joint(size_t)
        size_t num_joints()

    cdef cppclass cppJoint "chain::Joint" [Scalar]:
        
        cppclass position_type:
            Scalar& operator[](size_t)

        cppclass orientation_type:
            Scalar& x()
            Scalar& y()
            Scalar& z()
            Scalar& w()

        position_type& get_position()
        orientation_type& get_orientation()
        Scalar& get_angle()
        void set_angle(Scalar)


cdef class Joint:
    cdef public JointChain joint_chain
    cdef public Py_ssize_t index

    def __init__(self, chain, index):
        self.joint_chain = chain
        self.index = index

    @property
    def position(self):
        # this can be done without a copy, but reference counting complicates
        # things a lot
        cdef double[:] pos = np.empty(3, dtype=np.double)
        pos[0] = self.joint_chain.cppchain_.get_joint(self.index).get_position()[0]
        pos[1] = self.joint_chain.cppchain_.get_joint(self.index).get_position()[1]
        pos[2] = self.joint_chain.cppchain_.get_joint(self.index).get_position()[2]
        return pos

    @property
    def orientation(self):
        # this can be done without a copy, but reference counting complicates
        # things a lot
        cdef double[:] pos = np.empty(4, dtype=np.double)
        pos[0] = self.joint_chain.cppchain_.get_joint(self.index).get_orientation().x()
        pos[1] = self.joint_chain.cppchain_.get_joint(self.index).get_orientation().y()
        pos[2] = self.joint_chain.cppchain_.get_joint(self.index).get_orientation().z()
        pos[3] = self.joint_chain.cppchain_.get_joint(self.index).get_orientation().w()
        return pos

    def get_angle(self):
        return self.joint_chain.cppchain_.get_joint(self.index).get_angle()

    def set_angle(self, double angle):
        self.joint_chain.cppchain_.get_joint(self.index).set_angle(angle)


cdef class JointChain:
    cdef cppJointChain[double] cppchain_
    cdef public object joints

    def __init__(self):
        self.joints = [Joint(self, 0)]

    def add_joint(self, position, orientation):
        cdef double[:] pos = np.asarray(position, dtype=np.double)
        cdef double[:] ori = np.asarray(orientation, dtype=np.double)

        if pos.shape[0] != 3:
            raise ValueError('position must be an array of length 3')
        if ori.shape[0] != 4:
            raise ValueError('orientation must be an array of length 4')

        self.cppchain_.add_joint(&pos[0], &ori[0])
        joint = Joint(self, len(self.joints))
        self.joints.append(joint)
        return joint

    def base(self):
        return self.joints[0]

    def gripper(self):
        return self.joints[-1]

    def num_joints(self):
        return self.cppchain_.num_joints()
