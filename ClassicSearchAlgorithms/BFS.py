class DFS():

    def dfs(graph, start):
        visited, stack = set(), [start]
        while stack:
            vertex = stack.pop()
            if vertex not in visited:
                visited.add(vertex)
                stack.extend(graph[vertex] - visited)
        return visited

    graph = dfs({'E', 'D', 'F', 'A', 'C', 'B'}, 'A') # {'E', 'D', 'F', 'A', 'C', 'B'}
    print(graph)