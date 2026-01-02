import flwr as fl
from flwr.server.strategy import FedAvg
from flwr.server import start_server, ServerConfig

def main():
    print("Starting server")
    print("Waiting for 2 clients to connect")

    strategy = FedAvg(
        min_fit_clients=2,
        min_available_clients=2,
        fraction_fit=1.0, #Train on 100% of available clients
    )

    start_server(
        server_address="0.0.0.0:8080",
        config = ServerConfig(num_rounds=3),
        strategy=strategy,
    )

if __name__ == "__main__":
    main()