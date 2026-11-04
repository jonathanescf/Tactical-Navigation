from src.orchestration.software import software
from src.orchestration.config import parameters

def main():
    tactical_nav = software(parameters)


if __name__ == "__main__":
    main()
