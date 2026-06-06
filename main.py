from src.software import software
from src.user_scripts.config import parameters

def main():
    tactical_nav = software(parameters)
    tactical_nav.launch()


if __name__ == "__main__":
    main()
