from glob import glob
import torch
from torch import nn, optim
import os
import json
from datetime import datetime
import torchvision
import torchvision.transforms as transforms
from torchvision.io import read_image
from torchvision.models import resnet50, ResNet50_Weights

transform = transforms.Compose(
    [transforms.ToTensor(),
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

batch_size = 4

DATETIME_FORMAT = '%H-%M-%S.%f'


class DataSet(torch.utils.data.Dataset):
    def __init__(self):
        runs = [entry[6:-5] for entry in glob('train/*.json')]
        self.labels = {}
        for run in runs:
            with open('train/' + run + '.json') as f:
                self.labels[run] = json.load(f)
                self.labels[run]['records'] = [(datetime.strptime(
                    k, DATETIME_FORMAT), v) for k, v in self.labels[run]['records']]
        self.list = []
        for run in runs:
            self.list = [(run, obj) for obj in os.listdir(
                'train/' + run) if os.path.isfile('train/' + run + '/' + obj)]

    def __getitem__(self, index):
        run, obj = self.list[index]
        return read_image('train/' + run + '/' + obj), self.get_label(run, obj)

    def __len__(self):
        return len(self.list)

    def get_label(self, run, obj):
        time = datetime.strptime(obj[:-4], DATETIME_FORMAT)
        label = self.labels[run]['base']
        for t, v in self.labels[run]['records']:
            if t <= time:
                label = v
        return label


classes = ('background', 'sign_circle', 'sign_triangle')


def get_model(pretrained=False):
    model = resnet50(
        weights=ResNet50_Weights.DEFAULT) if pretrained else resnet50()
    model.fc = torch.nn.Linear(model.fc.in_features, 3)
    nn.init.xavier_uniform_(model.fc.weight)
    model.cuda()
    return model


def finetune(net, batch_size, learning_rate, epochs, resume=None):
    trainloader = torch.utils.data.DataLoader(DataSet(), batch_size=batch_size,
                                              shuffle=True, num_workers=2)
    criterion = nn.CrossEntropyLoss()
    params_1x = [param for name, param in net.named_parameters()
                 if name not in ["fc.weight", "fc.bias"]]
    trainer = torch.optim.SGD([{'params': params_1x},
                               {'params': net.fc.parameters(),
                                'lr': learning_rate * 10}],
                              lr=learning_rate, weight_decay=0.001)
    if resume:
        checkpoint = torch.load(resume)
        net.load_state_dict(checkpoint['model_state_dict'])
        trainer.load_state_dict(checkpoint['optimizer_state_dict'])
    for epoch in range(epochs):  # loop over the dataset multiple times
        running_loss = 0.0
        acc_count = 0
        total_count = 0
        for i, data in enumerate(trainloader):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data

            # zero the parameter gradients
            trainer.zero_grad()

            # forward + backward + optimize
            outputs = net(inputs.float().cuda())
            labels = labels.cuda()
            loss = criterion(outputs, labels)
            loss.backward()
            trainer.step()

            # print statistics
            running_loss += loss.item()
            acc_count += torch.sum(outputs.max(1).indices == labels)
            total_count += len(labels)
            if i % 10 == 9:
                print(
                    f'[{epoch + 1}, {i + 1:5d}] loss: {running_loss / 10:.3f}, acc: {acc_count/total_count:.5f}')
                running_loss = 0.0
        if epoch % 10 == 9:
            torch.save({
                'epoch': epoch,
                'model_state_dict': net.state_dict(),
                'optimizer_state_dict': trainer.state_dict(),
                'loss': loss,
            }, f"model-{epoch}.pth")
    print('Finished Training')


def load_checkpoint(filepath):
    net = resnet50()
    net.fc = torch.nn.Linear(net.fc.in_features, 3)
    checkpoint = torch.load(filepath)
    net.load_state_dict(checkpoint['model_state_dict'])
    return net


def predict(net, img):
    net.eval()
    return net(read_image(img).float().unsqueeze(0))
