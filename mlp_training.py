import onnx
import os
import torch
import torchvision
from torch import nn

# 九分类
'''
1
2
3
4
5
outpost
guard
base
negative'''

class MLP(nn.Module):

    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Flatten(),
            nn.Linear(20 * 28 * 1, 120),       #120个神经元，每个神经元都对应着一个长度为 560 的权重向量
            nn.ReLU(),
            nn.Linear(120, 84),
            nn.ReLU(),
            nn.Linear(84, 9)
        )

    def forward(self, x):
        return self.layers(x)


def save_model(model):
    # Save as onnx
    dummy_input = torch.randn(1, 20, 28, 1)
    torch.onnx.export(model, dummy_input, "mlp.onnx")

    # Check onnx
    onnx_model = onnx.load("mlp.onnx")
    onnx.checker.check_model(onnx_model)
    print(onnx.helper.printable_graph(onnx_model.graph))


# Init model
model = MLP()
print(model, "\n")

# Load data from folder
dataset = torchvision.datasets.ImageFolder(
    root=os.path.join(os.path.dirname(__file__), 'datasets'),
    transform=torchvision.transforms.Compose([
        torchvision.transforms.Grayscale(),
        torchvision.transforms.RandomAffine(
            degrees=(-5, 5), translate=(0.08, 0.08), scale=(0.9, 1.1)),
        torchvision.transforms.ToTensor(),
        torchvision.transforms.RandomErasing(
            scale=(0.02, 0.02))
    ]))
print(dataset, "\n")

# Show label names
print("classes:\n", dataset.classes, "\n")

# Split dataset into train and test (8:2) 划分数据集
train_dataset, test_dataset = torch.utils.data.random_split(
    dataset, [int(len(dataset) * 0.8), len(dataset) - int(len(dataset) * 0.8)])

# Define loss function and optimizer
loss_fn = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

# Define data loader
train_loader = torch.utils.data.DataLoader(
    train_dataset, batch_size=100, shuffle=True)
test_loader = torch.utils.data.DataLoader(
    test_dataset, batch_size=100, shuffle=True)

# Train and evaluate
for epoch in range(5):
    for batch, (x, y) in enumerate(train_loader):
        # Forward
        y_pred = model(x)

        # Compute loss
        loss = loss_fn(y_pred, y)

        # Backward
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # Print loss
        if batch % 100 == 0:
            print(f'Epoch: {epoch}, Batch: {batch}, Loss: {loss.item()}')

    # Evaluate
    with torch.no_grad():
        correct = 0
        total = 0
        for x, y in test_loader:
            y_pred = model(x)
            _, predicted = torch.max(y_pred.data, 1)
            total += y.size(0)
            correct += (predicted == y).sum().item()
        print(f'Epoch: {epoch}, Accuracy: {100 * correct / total}%')

    # Save model on each epoch 后来的会覆盖之前的，最终只留下最后一次的
    save_model(model)
    print("\n")
