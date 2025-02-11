# **Voronoi Fracture on 3D Meshes in Real Time**

### **Fundamentals of Voronoi**

The Voronoi method, known and referred to as Voronoi diagrams or Voronoi tessellation, is a mathematical and geometric concept used in various fields, including but not limited to geography, astrophysics, chemistry, and of course computer graphics. It is named after the Russian mathematician Georgy Voronoi, who first introduced the concept in 1908.  
In my case I will explore the use of Voronoi tessellation as it relates to real-time fracturing of a 3D mesh. The Voronoi fracture method is a powerful tool for simulating the breaking or shattering of objects. It involves dividing a space into regions based on distance to a set of points, creating a pattern that mimics the natural fracture lines seen in materials like glass or rock. In video games, this technique allows for the creation of dynamic, destructible environments that respond realistically to player actions or other in-game events.  
<br/>The Voronoi method is a way to partition a space into regions based on the proximity to a set of points. These regions are called Voronoi cells. I believe it is best explained in 2D.
<br/>![image](https://github.com/user-attachments/assets/37f6df7c-868b-4e2b-abc5-1af82c7dbcd8)

<br/>Each feature point commands a region of space, if you were to pick any point within that region the closest feature point to it would be the feature point commanding that cell. This is the basic principle of the Voronoi pattern. One way to look at it if at first you find it hard to understand is by picking out two feature points whose cells border each other and draw a parallel line equidistant between them, this way you will quickly see the distance-based boundaries of Voronoi illuminate.

### **Application in Video Games**

Incorporating Voronoi fracture into video games revolutionizes the player's interaction with the environment. It enables the creation of scenes where buildings crumble dynamically under stress, or objects shatter in a realistic manner. This not only enhances visual realism but also opens up new gameplay possibilities, such as creating new paths or using the environment strategically.  
<br/>Voronoi fractures are typically used in computer graphics primarily in relation to texturing, the natural pattern that result is perfect for animal hides and even cobblestones. A bonus in the fast-paced industry of game development is than simply by changing the distribution of the feature points you can achieve wildly different results.  
Voronoi fracturing has also been a staple of computer graphics for decades now, it is one of the most prolific destruction techniques in film as well as games. The beauty of Voronoi fracturing lies in the feature point distribution. In the case of a bullet impacting an object, you would be able to algorithmically scatter the feature points biasing towards the impact point of the bullet. This way the resulting chunks after the fracture would be smaller and more numerous near the bullets impact point, and in turn parts further away would be much larger, thereby simulating the distribution of force over the object.  
You can go even further with the distribution of feature points by scattering them in ways that means the resulting fracture pieces resemble a specific material. For example, when fracturing glass, by scattering feature points in close to uniform concentric circles or other uniform patterns, the resulting fractures will look like the geometric and uniform pieces we expect from broken glass. Whereas wood often splinters in long, thin pieces so distributing the feature points strategically in bands along the intended grain to promote these structures is common. Additionally with wood you can add a degree of noise to certain areas to emulate knots and warps in the wood.

### **Technical Implementation**

Implementing Voronoi fracture in real-time scenarios is technically demanding. The major drawback of Voronoi is the computational cost, not only of calculating the initial fracture but also creating brand new meshes from the resulting tessellation. In film this is hardly an issue as everything is pre-rendered but in games, where there is major consumer pressure for performance, having a real-time dynamic fracturing event while running the rest of the game at the same time is a huge task. For this reason, we do not actually do the fracturing dynamically in real-time, instead we do our best to create the illusion that this is what is happening.  
The methodology involves pre-calculating the fracture patterns and dynamically applying them during gameplay. This requires efficient algorithms to manage the computational load, ensuring the gaming experience remains fluid and responsive. Additionally, considerations like mesh generation, collision detection, and rendering are integral to achieving a seamless integration of this technique into the game's physics engine.

### **Conclusion**

By harnessing the power of Voronoi tessellation, game developers can introduce dynamic, destructible environments that respond authentically to player actions and in-game events. Whether it's buildings crumbling under stress, objects shattering realistically, or the creation of intricate textures resembling various materials, Voronoi fractures have become an indispensable tool in the game developer's arsenal.

The technical challenges of implementing Voronoi fracture in real-time scenarios are indeed formidable, requiring clever algorithms and optimizations to maintain performance while delivering a captivating gaming experience. Nevertheless, the rewards are substantial, as players can now engage with game worlds that react and evolve, offering a level of immersion and dynamism previously thought unattainable.

# **My Voronoi Fractures**

My implementation of Voronoi is based around the subdivision of space into voxels. I then run through the voxels one by one and get the distance between it and each of the feature points. If a voxel is equidistant to two feature points (within a threshold) then it makes up a Voronoi cell face, if equidistant to three feature points it is part of a Voronoi cell edge, and if equidistant to four feature points it is listed as a potential vertex of a Voronoi cell. This logic makes up the basics of the fracturing in my case.  

I started by making a cube and within it I placed a scattering of feature points, shown here in green. I also picked out the corners of the cube, shown here in red, as these are the first known vertices.
<br/>![image](https://github.com/user-attachments/assets/cf9f1954-46eb-4cb0-8c7d-db18373b42fe)

<br/>I then subdivided the space into voxels. My voxels are a custom struct containing the following information: position (top left), position (centre), size, master feature points (the feature point or points that this voxel belongs to.
<br/> ![image](https://github.com/user-attachments/assets/ef4a02d3-386f-45ff-b661-46c93a3d6b00)

<br/>The first order of business was to create the logic for determining whether a voxel was equidistant to two feature points. In the development screenshot above I have simply only added a new voxel if it is equidistant to two feature points and so they are the only ones in the list of voxels to be displayed.
<br/>![image](https://github.com/user-attachments/assets/b6b6a2a8-d6f0-473a-b3ab-f9a2a2803a8b)

<br/>Next I added logic for determining whether a voxel was equidistant to three feature points, highlighted here in red. During this early stage of development I was using a technique that I have since discarded which results in these multiple overlapping planes, I will explain this later on but for now I will continue discussing the development chronologically. 
<br/> ![image](https://github.com/user-attachments/assets/94b7262f-af8a-455c-8917-3400da783e56)

<br/>In this I have highlighted plane voxels by different colours depending on the first master feature point they are controlled by to visualise it better for debugging. At this stage I have added culling logic to stop the overlapping planes. The now archaic methodology to do this was by assuming that if a voxel is closer to another feature point than it was to either of its masters then it should be culled. As you can see, to a certain degree this logic did provide satisfactory results.
<br/> ![image](https://github.com/user-attachments/assets/a50eb917-a889-484f-8cc7-f0b02f9b9488)

<br/>Here I ran a test with more feature points and a much smaller voxel size to stress test my implementation. I have highlighted this feature point’s associated voxels in green to better visualise the boundaries of the Voronoi cell. In this you can make out a small tear in the plane and this was the crux of my culling logic.
<br/> ![image](https://github.com/user-attachments/assets/48c6979c-7b7e-4af4-aaf1-fb4d687df974)

<br/>I have provided the first image (see below) as an extreme example of the tearing in my Voronoi structure. As a reminder what I was doing was checking the equidistants to feature points and then rejecting voxels if they were closer to another feature point than their masters. This issue often manifested itself as cuts such as in the second image. These missing voxels have been rejected because they are technically closer to another feature point than their masters, but that feature point is not able to be their master because it is penned in by its Voronoi cell. As the plane goes outwards from its centre the raw distance between each voxel in the plane and its masters increases and this allows my rudimentary distance check with non-master feature points to fail at certain distances.
<br/> ![image](https://github.com/user-attachments/assets/3168b61f-b4d1-469d-b656-860c41337e84) ![image](https://github.com/user-attachments/assets/8ee8b03c-dc7a-4f36-88b4-5515733dec5e)

<br/>This is a random screenshot (see below) from a later date after I had fixed the hole issue. The new logic is beautifully simple, I get the distance from the current voxel I am checking to its closest feature point, then I find any other feature points that are equidistant. In hindsight my implementation was a lapse in judgement after being told that voxels equidistant to _x_ feature points equals _y_. So I got all voxels that were equidistant to _x_ and then when that didn’t work made a boiler plate solution to reject the unwanted voxels. After going back to the drawing board this new logic makes far more sense and as an added bonus is computationally much cheaper.
<br/> ![image](https://github.com/user-attachments/assets/df9cbe3d-7809-431f-8b14-dd9b812d5435)

<br/>Next, I needed to find the vertices of the Voronoi cell. This was by far the hardest part of the task; the initial principle is easy: find voxels that are equidistant to four feature points. However, this results in far more vertices than one would expect which in turn makes the later step of mesh building much more taxing. In the vast majority of cases the abundance of vertices can be thought of as the system finding the same vertex multiple times simply because the distance happens to be even. The task therefore is to pick only one of these vertices and reject the others.
<br/> ![image](https://github.com/user-attachments/assets/602b65f9-1563-4d3a-89ee-25f09b9eb4b4)

<br/>In the above image the accepted vertices are coloured in blue and the potential but ultimately rejected vertices are coloured in red. This image is still from development and still contains vertex artefacts but is a good example of some particular methodology. Namely, potential vertices are first grouped and then one from the group is picked, the one picked is simply the first one to be added which causes the vertices to be biased to the bottom of the potential group as an artefact of my generation.  
Special cases have to be made for creating vertices on the edge of the bounding cube when the plane and edge voxels intersect the bounding cube, this is done by slightly overflowing the voxel grid and checking when a voxels x,y,z is equal to one or more of the corners x,y,z co-ordinates. Similar grouping logic is then applied.
