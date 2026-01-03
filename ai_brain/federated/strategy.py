"""
@file strategy.py
@description
Custom federated learning strategy for Robo-Swarm-Fed.

Extends FedAdagrad to allow:
- Metadata inspection
- Custom logging hooks
- Future robustness logic

Phase 3 Policy:
- Behaves exactly like FedAdagrad
- No filtering or weighting yet
- Hooks are present but inactive
"""

from typing import List, Tuple, Dict, Optional, Union

from flwr.server.strategy import FedAdagrad
from flwr.common import Parameters, FitRes, Scalar
from flwr.server.client_proxy import ClientProxy


class CustomFedAdagrad(FedAdagrad):
    """
    Custom FL strategy wrapper.

    IMPORTANT:
    - Strategy does NOT control execution
    - Strategy ONLY defines aggregation behavior
    """

    def aggregate_fit(
        self,
        server_round: int,
        results: List[Tuple[ClientProxy, FitRes]],
        failures: List[Union[Tuple[ClientProxy, FitRes], BaseException]],
    ) -> Tuple[Optional[Parameters], Dict[str, Scalar]]:

        # ----------------------------------------------------------
        # Inspect client metadata (if provided)
        # ----------------------------------------------------------
        # This hook is intentionally passive in Phase 3.
        for _, res in results:
            if "train_metadata" in res.configs:
                # Placeholder for future robustness / filtering logic
                pass

        # ----------------------------------------------------------
        # Default FedAdagrad aggregation
        # ----------------------------------------------------------
        return super().aggregate_fit(server_round, results, failures)
