import os
os.environ['OMP_NUM_THREADS'] = '1'
import sys

project_base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.append(project_base_dir)

from planning.run_grasp_arm_gen import run_grasp_arm_gen
from utils.parallel import parallel_execute


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('--assembly-dir', type=str, required=True)
    parser.add_argument('--log-dir', type=str, required=True)
    parser.add_argument('--gripper', type=str, default='kuka', choices=['kuka', 'panda', 'robotiq-85', 'robotiq-140'], help='gripper type')
    parser.add_argument('--arm', type=str, default='kuka')
    parser.add_argument('--ft-sensor', type=str, default='none', choices=['none', 'all', 'move', 'hold'], help='force torque sensor installed')
    parser.add_argument('--max-n-grasp', type=int, default=None, help='maximum number of grasps per part')
    parser.add_argument('--n-surface-pt', type=int, default=200, help='number of surface point samples for generating antipodal pairs')
    parser.add_argument('--n-angle', type=int, default=10, help='number of grasp angle samples')
    parser.add_argument('--antipodal-thres', type=float, default=0.95)
    parser.add_argument('--ik-optimizer', type=str, default='least_squares', help='IK optimizer')
    parser.add_argument('--ik-regularization', type=float, default=1.0, help='IK regularization')
    parser.add_argument('--offset-delta', type=float, default=0.0)
    parser.add_argument('--reduced-limit', type=float, default=0.1, help='reduced joint limit in percentage')
    parser.add_argument('--num-proc', type=int, default=1, help='number of processes')
    parser.add_argument('--inner-num-proc', type=int, default=1)
    parser.add_argument('--seed', type=int, default=0, help='random seed')
    parser.add_argument('--verbose', action='store_true', default=False, help='verbose')
    args = parser.parse_args()

    os.makedirs(args.log_dir, exist_ok=True)

    worker_args = []
    for subdir_name in os.listdir(args.assembly_dir):
        sub_assembly_dir = os.path.join(args.assembly_dir, subdir_name)
        if os.path.isdir(sub_assembly_dir):
            sub_log_dir = os.path.join(args.log_dir, subdir_name)
            worker_args.append((sub_assembly_dir, sub_log_dir, args.gripper, args.arm, args.ft_sensor, args.seed, args.n_surface_pt, args.n_angle, args.antipodal_thres, args.ik_optimizer, args.ik_regularization, args.offset_delta, args.reduced_limit, args.max_n_grasp, args.inner_num_proc, False))
    try:
        for _ in parallel_execute(run_grasp_arm_gen, worker_args, num_proc=args.num_proc):
            pass
    except KeyboardInterrupt:
        exit()
