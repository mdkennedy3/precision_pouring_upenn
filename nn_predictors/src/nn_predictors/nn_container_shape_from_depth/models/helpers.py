import torch.nn as nn

def get_nonlinearity(nonlinearity):
    ''' Looks up the correct nonlinearity to use
    '''
    if nonlinearity == 'LeakyReLU':
        return nn.LeakyReLU()
    elif nonlinearity == 'ReLU':
        return nn.ReLU()
    else:
        raise NotImplementedError('Invalid nonlinearity')
