import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class TrussOptimizer:
    def __init__(self, num_nodes, load_positions, load_magnitudes):
        self.num_nodes = num_nodes
        self.load_positions = np.array(load_positions)
        self.load_magnitudes = np.array(load_magnitudes)
        self.material_properties = self.set_material_properties()
        
    def set_material_properties(self):
        # placeholder values for material properties
        return {
            'density': 2700,  # kg/m^3
            'yield_strength': 250e6,  # Pa
            'elastic_modulus': 70e9,  # Pa
            'safety_factor': 1.5
        }
        
    def generate_truss(self, params):
        # convert flat array to 3D array of node positions
        node_positions = params.reshape(-1, 3)
        
        # create connections between all nodes
        connections = []
        for i in range(self.num_nodes):
            for j in range(i+1, self.num_nodes):
                connections.append((i, j))
        
        return node_positions, connections
    
    def calculate_member_forces(self, node_positions, connections):
        num_members = len(connections)
        A = np.zeros((3*self.num_nodes, num_members))
        b = np.zeros(3*self.num_nodes)
        
        # force equilibrium equations
        for i, (n1, n2) in enumerate(connections):
            dx = node_positions[n2][0] - node_positions[n1][0]
            dy = node_positions[n2][1] - node_positions[n1][1]
            dz = node_positions[n2][2] - node_positions[n1][2]
            length = np.sqrt(dx**2 + dy**2 + dz**2)
            
            direction = np.array([dx, dy, dz]) / length
            
            A[3*n1:3*n1+3, i] = direction
            A[3*n2:3*n2+3, i] = -direction
        
        # apply external loads
        for pos, mag in zip(self.load_positions, self.load_magnitudes):
            node = np.argmin(np.sum((node_positions - pos)**2, axis=1))
            b[3*node:3*node+3] += mag
        
        # solve for member forces
        member_forces = np.linalg.lstsq(A, b, rcond=None)[0]
        
        return member_forces
    
    def calculate_truss_weight(self, node_positions, connections, member_forces):
        total_weight = 0
        for (n1, n2), force in zip(connections, member_forces):
            length = np.linalg.norm(node_positions[n2] - node_positions[n1])
            # calculation of required area
            required_stress = self.material_properties['yield_strength'] / self.material_properties['safety_factor']
            area = abs(force) / required_stress
            volume = length * area
            weight = volume * self.material_properties['density']
            total_weight += weight
        return total_weight
    
    def objective_function(self, params):
        node_positions, connections = self.generate_truss(params)
        member_forces = self.calculate_member_forces(node_positions, connections)
        truss_weight = self.calculate_truss_weight(node_positions, connections, member_forces)
        
        # Objective: minimize the truss weight
        return truss_weight
    
    def optimize(self):
        # Initial guess: evenly spaced nodes in a 3D grid
        x = np.linspace(0, 1, int(np.cbrt(self.num_nodes)))
        y = np.linspace(0, 1, int(np.cbrt(self.num_nodes)))
        z = np.linspace(0, 1, int(np.cbrt(self.num_nodes)))
        initial_guess = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3).flatten()
        
        # Optimization
        result = minimize(self.objective_function, initial_guess, method='SLSQP')
        
        # Get optimized truss
        optimized_nodes, connections = self.generate_truss(result.x)
        optimized_forces = self.calculate_member_forces(optimized_nodes, connections)
        optimized_weight = self.calculate_truss_weight(optimized_nodes, connections, optimized_forces)
        
        return optimized_nodes, connections, optimized_forces, optimized_weight
    
    def visualize_truss(self, nodes, connections, forces):
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # plot nodes
        ax.scatter(nodes[:, 0], nodes[:, 1], nodes[:, 2], c='r', s=50)
        
        # plot members
        for (n1, n2), force in zip(connections, forces):
            x = [nodes[n1, 0], nodes[n2, 0]]
            y = [nodes[n1, 1], nodes[n2, 1]]
            z = [nodes[n1, 2], nodes[n2, 2]]
            
            # color based on force (blue for compression, red for tension)
            color = 'b' if force < 0 else 'r'
            linewidth = 1 + 3 * abs(force) / max(abs(forces))  # adjust line width based on force magnitude
            
            ax.plot(x, y, z, color=color, linewidth=linewidth)
        
        # plot loads
        for pos, mag in zip(self.load_positions, self.load_magnitudes):
            ax.quiver(pos[0], pos[1], pos[2], mag[0], mag[1], mag[2], 
                      color='g', length=0.1, normalize=True)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Optimized 3D Truss Structure')
        
        plt.tight_layout()
        plt.show()

# example usage
num_nodes = 8  # 2x2x2 grid
load_positions = [(0.5, 0.5, 0.5), (0.7, 0.3, 0.8)]
load_magnitudes = [(0, 0, -1000), (-500, -500, -500)]

optimizer = TrussOptimizer(num_nodes, load_positions, load_magnitudes)
nodes, connections, forces, weight = optimizer.optimize()

print("Optimized node positions:")
print(nodes)
print("\nConnections:")
print(connections)
print("\nMember forces:")
print(forces)
print(f"\nOptimized truss weight: {weight:.2f} kg")

# visualize the optimized truss
optimizer.visualize_truss(nodes, connections, forces)