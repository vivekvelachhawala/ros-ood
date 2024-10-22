import os
import torch
from torch import nn
from torch.utils.data import DataLoader
import torchvision.transforms as transforms
from torchvision.datasets import ImageFolder
from ood_module_betav import Encoder  # Import the Encoder class

def main():
    # Define your device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    # Data transformations and dataset initialization
    transform = transforms.Compose([
        transforms.Resize((117, 149)),
        transforms.ToTensor(),
    ])

    dataset = ImageFolder(root='/home/lms/extracted_images (copy)', transform=transform)
    train_loader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Initialize the model
    model = Encoder(device).to(device)
    print("Model initialized.")

    # Define the optimizer and loss function
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-5)
    criterion = nn.MSELoss()  # Use a suitable loss function

    # Training loop
    num_epochs = 10
    for epoch in range(num_epochs):
        model.train()
        for images, _ in train_loader:
            images = images.to(device)

            # Forward pass
            optimizer.zero_grad()
            outputs = model(images)

        print(f"Epoch [{epoch + 1}/{num_epochs}] completed.")

    # Define the model path
    model_path = "/home/lms/ros-ood/packages/ros_package/src/trained_encoder.pth"

    # Save the trained model with a structured format
    torch.save({'model_state_dict': {'encoder.' + k: v for k, v in model.state_dict().items()}}, model_path)
    print(f"Model saved at {model_path}.")

    # Save the _config file
    config_path = model_path + "_config"
    config_data = {
        "topz": [1, 2, 3],  # Replace with actual topz indices as per your needs
        "threshold": 0.5    # Replace with the appropriate threshold value
    }

    # Write the config data to a file
    with open(config_path, 'w') as f:
        for key, value in config_data.items():
            if isinstance(value, list):
                for item in value:
                    f.write(f"{key}:{item}\n")
            else:
                f.write(f"{key}:{value}\n")

    print(f"Config file saved at {config_path}")

if __name__ == "__main__":
    main()
