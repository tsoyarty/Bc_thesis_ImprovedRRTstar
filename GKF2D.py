import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde
from mpl_toolkits.mplot3d import Axes3D

from sklearn.neighbors import KernelDensity
# Generate some sample data from a binormal distribution
# np.random.seed(0)
# pts = np.random.multivariate_normal(mean=[0, 0], cov=[[1, 0.9], [0.9, 1]], size=500)
# print(pts)

def plot_density(pts):
    pts = np.array(pts)
    # print(type(pts), type(pts[0]))
    # KDE using scipy.stats.gaussian_kde
    kde = gaussian_kde(pts.T) 
    
    
    # # Create Gaussian KDE estimator
    # kde2 = KernelDensity(kernel='gaussian').fit(pts)
    
    # # Bandwidth matrix
    # bandwidth_matrix = kde2.bandwidth
    # print("Bandwidth Matrix (H):", bandwidth_matrix)
    
    # Define the grid of points for visualization
    x = np.linspace(0, 150, 40)
    y = np.linspace(0, 150, 40)
    X, Y = np.meshgrid(x, y)
    positions = np.vstack([X.ravel(), Y.ravel()])
    
    # Calculate the KDE values on the grid
    Z = np.reshape(kde(positions).T, X.shape)
    
    
    #------------------------------------------------------
    #------------------------------------------------------
    
    # New point coordinates
    new_point = np.array([87.3,59.6])
    
    # Evaluate KDE at the new point
    density_at_new_point = kde(new_point)
    
    print("Density at new point:", density_at_new_point)
    
    #------------------------------------------------------
    #------------------------------------------------------
    
    
    # Plotting the KDE
    fig = plt.figure(figsize=(100,60))
    
    # Plot using matplotlib's contourf
    ax1 = fig.add_subplot(121)
    ax1.contourf(X, Y, Z, cmap='viridis')
    ax1.scatter(pts[:, 0], pts[:, 1], alpha=0.5, color='blue')
    # ax1.set_title('Using gaussian_kde (contourf)')
    # ax1.set_xlabel('X')
    # ax1.set_ylabel('Y')
    # ax1.legend([])
    
    # Plot using matplotlib's 3D surface plot
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none')
    ax2.scatter(pts[:, 0], pts[:, 1], zs=0, zdir='z', alpha=0.5, color='blue')
    # ax2.set_title('Using gaussian_kde (3D surface)')
    # Increase fontsize of axes labels
    ax2.set_xlabel('X',       fontsize=0)
    ax2.set_ylabel('Y',       fontsize=0)
    ax2.set_zlabel('Density', fontsize=0)
    
    # Increase fontsize of tick labels
    ax2.tick_params(axis='x', labelsize=0)
    ax2.tick_params(axis='y', labelsize=0)
    ax2.tick_params(axis='z', labelsize=0)
    
    # ax2.set_xticks([])
    # ax2.set_yticks([])
    # ax2.set_zticks([])
    
    # ax2.legend([])
    
    # plt.savefig('/home/tsoyarty/Desktop/Bc_work/Documentation/figChap4/GKDEobsNew.pdf', bbox_inches='tight', pad_inches=0)
    plt.tight_layout()
    plt.show()
    
pts = [
    np.array([90, 22]), np.array([111, 25]), np.array([30, 99]), np.array([73, 113]), np.array([65, 90]),
    np.array([135, 100]), np.array([142, 48]), np.array([89, 116]), np.array([149, 47]), np.array([116, 93]),
    np.array([36, 112]), np.array([26, 109]), np.array([8, 142]), np.array([84, 22]), np.array([106, 86]),
    np.array([125, 10]), np.array([79, 29]), np.array([115, 96]), np.array([80, 30]), np.array([69, 102]),
    np.array([48, 11]), np.array([22, 54]), np.array([60, 143]), np.array([133, 135]), np.array([122, 83]),
    np.array([85, 12]), np.array([124, 119]), np.array([129, 30]), np.array([145, 26]), np.array([125, 119]),
    np.array([131, 41]), np.array([115, 16]), np.array([2, 116]), np.array([106, 3]), np.array([48, 149]),
    np.array([122, 102]), np.array([100, 140]), np.array([85, 132]), np.array([147, 137]), np.array([24, 72]),
    np.array([43, 148]), np.array([47, 111]), np.array([116, 131]), np.array([14, 46]), np.array([69, 10]),
    np.array([25, 47]), np.array([3, 115]), np.array([81, 122]), np.array([65, 143]), np.array([143, 94]),
    np.array([50, 90]), np.array([5, 65]), np.array([99, 81]), np.array([86, 75]), np.array([31, 30]),
    np.array([123, 84]), np.array([132, 127]), np.array([20, 43]), np.array([127, 124]), np.array([46, 125]),
    np.array([96, 119]), np.array([74, 120]), np.array([5, 36]), np.array([146, 13]), np.array([102, 49]),
    np.array([59, 70]), np.array([126, 125]), np.array([146, 22]), np.array([78, 121]), np.array([17, 60]),
    np.array([127, 92]), np.array([90, 120]), np.array([40, 43]), np.array([21, 105]), np.array([22, 126]),
    np.array([117, 2]), np.array([2, 99]), np.array([43, 46]), np.array([31, 34]), np.array([102, 132]),
    np.array([146, 10]), np.array([51, 37]), np.array([82, 115]), np.array([66, 37]), np.array([20, 8]),
    np.array([147, 37]), np.array([121, 149]), np.array([67, 16]), np.array([127, 149]), np.array([145, 13]),
    np.array([123, 109]), np.array([9, 42]), np.array([117, 121]), np.array([143, 2]), np.array([24, 37]),
    np.array([48, 12]), np.array([64, 78]), np.array([16, 54]), np.array([25, 116]), np.array([128, 83]),
    np.array([54, 32]), np.array([70, 110]), np.array([111, 137]), np.array([118, 3]), np.array([65, 119]),
    np.array([50, 85]), np.array([129, 109]), np.array([147, 44]), np.array([85, 4]), np.array([97, 1]),
    np.array([14, 116]), np.array([93, 101]), np.array([141, 24]), np.array([95, 93]), np.array([103, 126]),
    np.array([30, 28]), np.array([91, 42]), np.array([1, 111]), np.array([42, 21]), np.array([8, 15]),
    np.array([32, 82]), np.array([16, 116]), np.array([123, 12]), np.array([20, 31]), np.array([137, 37]),
    np.array([47, 29]), np.array([56, 98]), np.array([115, 31]), np.array([65, 37]), np.array([4, 13]),
    np.array([125, 99]), np.array([73, 143]), np.array([38, 6]), np.array([22, 47]), np.array([44, 29]),
    np.array([131, 128]), np.array([41, 105]), np.array([131, 134]), np.array([128, 44]), np.array([131, 65]),
    np.array([82, 123]), np.array([95, 116]), np.array([139, 130]), np.array([129, 18]), np.array([3, 88]),
    np.array([21, 97]), np.array([95, 101]), np.array([9, 72]), np.array([43, 112]), np.array([79, 130]),
    np.array([108, 35]), np.array([134, 23]), np.array([88, 47]), np.array([84, 142]), np.array([65, 22]),
    np.array([99, 137]), np.array([42, 136]), np.array([111, 4]), np.array([12, 140]), np.array([72, 91]),
    np.array([76, 116]), np.array([129, 142]), np.array([67, 78]), np.array([67, 77])]


#OBSTACLE POINTS
pts = [
    np.array([46, 61]), np.array([83, 60]), np.array([76, 66]), np.array([47, 52]), np.array([71, 53]),
    np.array([47, 58]), np.array([75, 66]), np.array([96, 57]), np.array([59, 55]), np.array([57, 57]),
    np.array([109, 53]), np.array([50, 64]), np.array([41, 68]), np.array([79, 52]), np.array([100, 54]),
    np.array([39, 53]), np.array([69, 59]), np.array([89, 66]), np.array([83, 64]), np.array([47, 66]),
    np.array([90, 59]), np.array([104, 65]), np.array([82, 65]), np.array([59, 53]), np.array([93, 64]),
    np.array([46, 52]), np.array([42, 58]), np.array([60, 56]), np.array([98, 67]), np.array([83, 54]),
    np.array([59, 66]), np.array([109, 63]), np.array([78, 57]), np.array([88, 66]), np.array([81, 50]),
    np.array([105, 53]), np.array([95, 64]), np.array([113, 57]), np.array([80, 51]), np.array([93, 62]),
    np.array([81, 59]), np.array([60, 59]), np.array([92, 63]), np.array([82, 67]), np.array([99, 54]),
    np.array([65, 56]), np.array([49, 57]), np.array([52, 54]), np.array([92, 57]), np.array([85, 57]),
    np.array([82, 64]), np.array([77, 51]), np.array([58, 60]), np.array([37, 58]), np.array([94, 60]),
    np.array([66, 60]), np.array([71, 54]), np.array([40, 67]), np.array([35, 58]), np.array([78, 54]),
    np.array([85, 54]), np.array([62, 51]), np.array([49, 63]), np.array([72, 67]), np.array([46, 57]),
    np.array([96, 54]), np.array([82, 58]), np.array([54, 65]), np.array([58, 61]), np.array([51, 66]),
    np.array([55, 56]), np.array([104, 67]), np.array([95, 63]), np.array([43, 63]), np.array([38, 60]),
    np.array([101, 67]), np.array([81, 54]), np.array([86, 68]), np.array([54, 64]), np.array([56, 64]),
    np.array([107, 68]), np.array([50, 69]), np.array([83, 50]), np.array([40, 53]), np.array([50, 68]),
    np.array([37, 64]), np.array([55, 50]), np.array([108, 56])
]



plot_density(pts)
