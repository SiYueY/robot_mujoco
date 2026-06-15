"""RoboCasa integration exceptions."""


class RoboCasaIntegrationError(RuntimeError):
    """Base exception for all RoboCasa integration errors."""


class RoboCasaDependencyError(RoboCasaIntegrationError):
    """Raised when RoboCasa, robosuite, MuJoCo, or another dependency is missing."""


class RoboCasaSceneConfigError(RoboCasaIntegrationError):
    """Raised when a RoboCasa scene configuration is invalid."""


class RoboCasaSceneGenerationError(RoboCasaIntegrationError):
    """Raised when RoboCasa fails to generate a kitchen scene."""


class RoboCasaMjcfAdaptationError(RoboCasaIntegrationError):
    """Raised when the generated RoboCasa MJCF cannot be adapted."""
