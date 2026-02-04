### **Description of the Program**

This program is a simplified simulation of the game "Risk" with a focus on object-oriented programming concepts. It pits two players—**the user** and **the computer**—against each other in a turn-based battle to control an army. Each player recruits an army of different unit types within a specified budget, then takes turns attacking and defending until one player is completely defeated.

The program is designed to teach students important programming concepts such as:
- **Classes and Objects**: Representing game units (`Footman`, `Archer`, `Knight`, and `Siege Machine`) and players using classes.
- **Inheritance and Polymorphism**: Defining unique unit behaviors using subclasses of a base class (`Unit`).
- **Encapsulation**: Managing player-specific information (e.g., budget, army) within the `Player` class.
- **List Operations**: Adding, removing, and filtering units in the army during recruitment and battle phases.
- **Randomization**: Introducing unpredictability in attacks and unit recruitment using the `random` module.
- **Game Logic**: Implementing a turn-based system with clear victory conditions.

By playing this game, students will learn how to:
1. Create and use Python classes.
2. Use object-oriented programming to manage complex game logic.
3. Combine data structures, randomness, and loops to simulate real-world scenarios.

---

### **Rules of the Game**

1. **Recruiting an Army**:
   - Each player begins with a fixed budget (e.g., 30 coins) to recruit an army.
   - Four unit types are available for recruitment:
     - **Footman**: Costs 1 coin, rolls 1 die, hits on 5+, health = 1.
     - **Archer**: Costs 2 coins, rolls 1 die, hits on 4+, health = 1.
     - **Knight**: Costs 3 coins, rolls 1 die, hits on 3+, health = 2.
     - **Siege Machine**: Costs 10 coins, rolls 2 dice, hits on 3+, health = 3.
   - Players can recruit multiple units but can only have up to **2 Siege Machines**.

2. **Battle Phases**:
   - Players take turns attacking and defending.
   - During a player's turn:
     - A specific unit type (rotating between **Siege Machines**, **Archers**, **Knights**, and **Footmen**) is selected for attack.
     - Each unit of the chosen type rolls dice to determine hits.
     - Hits are calculated based on the unit's hit threshold (e.g., rolls of 5+ for Footmen).
     - The defending player takes damage by randomly selecting units to absorb hits.
     - Units with zero health are removed from the game.

3. **Victory Condition**:
   - The game ends when one player's army is entirely defeated (i.e., no units remain alive).
   - If the user's army survives while the computer's army is defeated, the user wins.
   - If the computer's army survives while the user's army is defeated, the computer wins.

4. **Special Mechanics**:
   - **Randomization in Recruitment**: Units are recruited randomly based on the player's remaining budget.
   - **Dynamic Turn Rotation**: The type of unit attacking changes in each turn, cycling through unit types in order: **Siege Machines → Archers → Knights → Footmen**.
   - **Damage Resolution**: Damage is applied randomly to units, ensuring unpredictability in gameplay.

---

### **Game Example Walkthrough**

1. **Start**:
   - The program begins by welcoming the user and asking for their name.
   - Both the user and the computer recruit armies within their budget.

2. **Army Recruitment**:
   - The user recruits units randomly based on the budget, with the composition displayed after recruitment. For example:
     ```
     Dr. Baek's Army Composition:
     Footman: 3
     Archer: 2
     Knight: 1
     Siege Machine: 1
     ```

3. **Battle**:
   - Turns alternate between the user and the computer.
   - Example:
     ```
     Dr. Baek's turn to attack!
     Archer rolls 4. Archer scores 1 hit(s).
     Footman rolls 5. Footman scores 1 hit(s).
     Dr. Baek dealt 2 total hits!

     Computer receives 2 total damage!
     Archer has been eliminated!
     ```

4. **Victory**:
   - The game ends when one player's army is completely eliminated:
     ```
     Computer is defeated! Dr. Baek wins!
     ```

---

### **Learning Outcomes**

1. **Object-Oriented Design**:
   - Understand how to use inheritance to create specialized unit types (`Footman`, `Archer`, `Knight`, and `Siege Machine`).
   - Learn encapsulation by organizing game-related information into reusable classes.

2. **Program Structure**:
   - Explore how multiple functions and classes work together to create a complete program.

3. **Game Logic**:
   - Understand how randomness and turn-based systems can be implemented programmatically.

4. **Debugging and Problem-Solving**:
   - Analyze and modify code to fix bugs and introduce new features.

---

### **Assignment Suggestions**

- **Extension 1**: Add new unit types with unique costs, health, and dice mechanics.
- **Extension 2**: Implement a custom recruitment phase where users manually choose units.
- **Extension 3**: Modify the victory condition to include a "total damage dealt" tie-breaker if both armies are defeated in the same turn.
- **Extension 4**: Add a graphical interface using a library like `tkinter` or `pygame`.

---

This description and the rules provide an engaging introduction to object-oriented programming while allowing students to have fun learning. Let me know if you'd like to refine any part!