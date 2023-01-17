class MultiCell():
    d = {}

    def __len__(self):
        return len(self.d)

    def len_subset(self, key):
        try: 
            return len(self.d[key])
        except KeyError:
            return 0

    def set(self, value, idx1, idx2):
        if idx1 in self.d:
            self.d[idx1][idx2] = value
        else:
            self.d[idx1] = { idx2 : value }

    def get(self, idx1, idx2):
        try: 
            return self.d[idx1][idx2]
        except KeyError:
            return None
