from train_options import TrainOptions
from shape_trainer import ShapeTrainer

if __name__ == '__main__':
    options = TrainOptions().parse_args()
    trainer = ShapeTrainer(options)
    trainer.train()