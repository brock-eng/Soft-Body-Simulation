# Soft-Body-Simulation

C++/OpenGL soft body simulation program using my <a href="https://github.com/brock-eng/Legit-Game-Engine">2D Game Engine</a>.  Uses the <a href="https://en.wikipedia.org/wiki/Soft-body_dynamics#Spring/mass_models">spring/mass physics model</a> for approximating soft body behavior.  The program uses a generated mesh of nodes and springs, evaluating the net force vector produced on each node (due to springs & gravity) per rendering sequence.  Only after calculating all forces does the program apply corresponding velocity to each node.

<img src="softbody.png">

## References
<ul>
   <li><a href="https://www.youtube.com/watch?v=kyQP4t_wOGI">'But How DO Soft Body Simulations Work?'</a>- Youtube</li>
</ul>
