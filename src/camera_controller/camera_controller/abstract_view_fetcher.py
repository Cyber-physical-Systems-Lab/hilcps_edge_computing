from abc import abstractmethod


# Todo for now I'm only using the IpFetcher,
# But extend here for the lab camera + computer camera
class AbstractViewFetcher:
    def __init__(self):
        super().__init__()
    

    @abstractmethod    
    def fetch_image(self):
        pass

