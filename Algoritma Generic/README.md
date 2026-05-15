# Algoritma Generic

Folder ini berisi implementasi pencarian graf/ruang status generik untuk keperluan perencanaan jalur.

File utama:
- `algoritma_genetika.py` — menyediakan fungsi `generic_search` yang mendukung strategi:
  - `astar` (A*)
  - `dijkstra` (Dijkstra)
  - `greedy` (Greedy Best-First)
  - `bfs` (Breadth-First Search)
  - `dfs` (Depth-First Search)

Contoh ringkas penggunaan:

```python
from generic_algorithm import generic_search

# node bisa berupa tuple koordinat atau objek status
start = (0, 0)

def is_goal(n):
    return n == (2, 2)

# neighbors harus mengembalikan list of (neighbor, cost)
def neighbors(n):
    x, y = n
    cand = [ (x+1,y), (x-1,y), (x,y+1), (x,y-1) ]
    valid = [ (nx,ny) for nx,ny in cand if nx >= 0 and ny >= 0 and nx <= 2 and ny <= 2 ]
    return [(v, 1.0) for v in valid]

path, cost = generic_search(start, is_goal, neighbors, heuristic=lambda n: 0.0, strategy='bfs')
print('Path:', path, 'Cost:', cost)
```

Jika Anda ingin saya mengintegrasikan fungsi ini ke `motion_planning.py` atau menambahkan contoh visualisasi, beri tahu saya.
