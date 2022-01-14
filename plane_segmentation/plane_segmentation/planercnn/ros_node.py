from options import parse_args
from config import InferenceConfig
import pickle as pkl

def main():
    pass


if __name__ == '__main__':
    args = parse_args()

    if args.dataset == '':
        args.keyname = 'evaluate'
    else:
        args.keyname = args.dataset
        pass
    args.test_dir = 'test/' + args.keyname

    if args.testingIndex >= 0:
        args.debug = True
        pass
    if args.debug:
        args.test_dir += '_debug'
        args.printInfo = True
        pass
    options = args
    with open('options.pkl', 'wb') as f:
        pkl.dump(options,f)
    config = InferenceConfig(options)
