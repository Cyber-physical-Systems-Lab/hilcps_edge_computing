from abc import abstractmethod



class AbstractViewFetcher:
    def __init__(self):
        super().__init__()
    

    @abstractmethod    
    def fetch_image(self):
        pass

