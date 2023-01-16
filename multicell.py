class MultiCell():
    d = {}

    def __len__(self):
        return len(self.d)

    def len_subset(self, key):
        return len(self.d[key])

    def set(self, value, idx1, idx2):
        if idx1 in self.d:
            self.d[idx1][idx2] = value
        else:
            self.d[idx1] = { idx2 : value }

    def get(self, idx1, idx2):
        return self.d[idx1][idx2]
