"""
Risk Game Module

Functions: main: Entry point for the game. Initialize and start the Risk game.

Author: Dr. Stan Baek, United Stated Air Force Academy
Date: 18 Jan 2025

**IMPORTANT DISCLAIMER** This code is intended solely for use within the ECE387 class at the United States Air Force Academy. Unauthorized sharing, distribution, or reproduction of this code is strictly prohibited.
"""

import random
from itertools import cycle

# Base class for all units
class Unit:
    """
    Represents a generic unit in the game.
    """

    def __init__(self, name: str, cost: int, health: int, hit_threshold: int):
        self.name = name
        self.cost = cost
        self.health = health
        self.hit_threshold = hit_threshold

    def roll_attack(self) -> int:
        roll = random.randint(1, 6)  # Roll a six-sided die
        print(f"{self.name} rolls {roll}.")
        return 1 if roll >= self.hit_threshold else 0
    

    def take_damage(self, damage: int) -> None:
        self.health = max(self.health - damage,0)
    

    def isalive(self) -> bool:
        if self.health > 0:
            return True
        else:
            return False

    def __str__(self) -> str:
        """Return a string representation of the unit."""
        return f"{self.name} (Health: {self.health})"


# Subclasses for specific unit types
class Footman(Unit):
    """Represents a Footman unit with specific attributes."""
    def __init__(self, name: str ="Footman"):
        super().__init__(name=name, cost=1, health=1, hit_threshold=5)


class Archer(Unit):
    """Represents an Archer unit with specific attributes."""
    def __init__(self, name: str ="Archer"):
        super().__init__(name=name, cost=2, health=1, hit_threshold=4)


class Knight(Unit):
    """Represents a Knight unit with specific attributes."""
    def __init__(self, name: str = "Knight"):
        super().__init__(name=name, cost=3, health=2, hit_threshold=3)


class SiegeMachine(Unit):
    """Represents a Siege Machine unit with specific attributes."""
    def __init__(self, name: str = "Siege Machine"):
        super().__init__(name=name, cost=10, health=3, hit_threshold=3)

    def roll_attack(self) -> int:
        hits = 0
        dice_1 = random.randint(1, 6)
        print(f"{self.name} rolls {dice_1}.")
        if dice_1 >= self.hit_threshold:
            hits =  hits + 1
            
        dice_2 = random.randint(1, 6)
        print(f"{self.name} rolls {dice_2}.")
        if dice_2 >= self.hit_threshold:
            hits = hits + 1
        
        return hits
        
        


class Player:
    """
    Represents a player in the game.
    Attributes:
        name: The name of the player.
        budget: The amount of coins available for recruiting units.
        army: A list of units the player has recruited.
    """

    MaxSiegeUnits = 2  # Maximum number of Siege Machines allowed per player

    def __init__(self, name, budget=100):
        self.name = name
        self.budget = budget
        self.army = []  # List of recruited units
        self.army_type = cycle((SiegeMachine, Archer, Knight, Footman))  # Cycle through unit types

    def recruit_units(self) -> None:

        unit_types = [Footman, Archer, Knight, SiegeMachine]
        siege_count = 0

        while self.budget > 0:
            unit_class = random.choice(unit_types)  # Randomly pick a unit type
            unit = unit_class()  # Create an instance of the unit
            if unit.cost <= self.budget:
                if isinstance(siege_count, SiegeMachine) and siege_count >= Player.MaxSiegeUnits:
                    continue
                self.army.append(unit)
                self.budget -= unit.cost
                if isinstance(unit, SiegeMachine):
                    siege_count += 1

        # Display the player's army composition after recruitment
        unit_counts = self.get_army_composition()
        print(f"\n{self.name}'s Army Composition:")
        for unit_type, count in unit_counts.items():
            print(f"{unit_type}: {count}")

    def get_army_composition(self) -> dict:
        """
        Count and return the number of each type of unit in the army.
        Returns:
            A dictionary with unit types as keys and their counts as values.
        """
        unit_counts = {
            "Footman": sum(1 for unit in self.army if isinstance(unit, Footman)),
            "Archer": sum(1 for unit in self.army if isinstance(unit, Archer)),
            "Knight": sum(1 for unit in self.army if isinstance(unit, Knight)),
            "Siege Machine": sum(1 for unit in self.army if isinstance(unit, SiegeMachine)),
        }
        return unit_counts

    def is_defeated(self) -> bool:
        """
        Check if the player has any units left alive.
        Returns:
            True if all units are dead; False otherwise.
        """
        return all(not unit.isalive() for unit in self.army)

    def attack(self, defender) -> None:

        print(f"\n{self.name}'s turn to attack!")

        total_hits = 0
        hits = 0
        army_type = next(self.army_type)  # Get the current unit type in the cycle

        for unit in self.army:
            if isinstance(unit, army_type) and unit.isalive():
                hits = unit.roll_attack()
                total_hits += hits
                print(f"{unit.name} (Health: {unit.health}) scores {hits} hit(s)!")

        print(f"{self.name} dealt {total_hits} total hits!")
        defender.resolve_damage(total_hits)

    def resolve_damage(self, total_damage: int) -> None:
        print(f"{self.name} receives {total_damage} total damage!")

        while total_damage > 0 and self.army:
            unit = random.choice(self.army)
            unit.take_damage(1) 
            total_damage -= 1
            print(f"{unit.name} takes 1 damage!")
            
            if unit.health == 0:
                self.army.remove(unit)
                print(f"\n{unit.name} (Health: {unit.health}) has been eliminated")
                
        
    def __str__(self):
        """
        Return a string representation of the player's army composition.
        """
        unit_counts = self.get_army_composition()
        return (f"{self.name}'s Army\n"
                f"Footmen: {unit_counts['Footman']}, Archers: {unit_counts['Archer']}, "
                f"Knights: {unit_counts['Knight']}, Siege Machines: {unit_counts['Siege Machine']}")


class Risk:
    """Represents the Risk game."""

    def __init__(self, name: str, budget: int = 30):
        self.user = Player(name=name, budget=budget)
        self.computer = Player(name="Computer", budget=budget)

        # Recruit units for both players
        self.user.recruit_units()
        self.computer.recruit_units()

        # Display initial armies
        print("\nInitial Armies:")
        print(self.user)
        print(self.computer)

    def play(self) -> None:
        """
        Simulate the battle between the user and the computer until one is defeated.
        """
        while not self.user.is_defeated() and not self.computer.is_defeated():
            # User attacks first
            self.user.attack(self.computer)
            if self.computer.is_defeated():
                print("\nComputer is defeated! You win!")
                break

            # Computer's turn to attack
            self.computer.attack(self.user)
            if self.user.is_defeated():
                print("\nYou are defeated! Computer wins!")
                break

        # Display final armies
        print("\nFinal Armies:")
        print(self.user)
        print(self.computer)


def test():
    hulk = Footman("Bruce Banner")
    blackwidow = Archer("Natasha Romanova")
    batman = Knight("Bruce Wayne")

    units = (hulk, blackwidow, batman)

    for _ in range(5):
        for unit in units:
            hit = unit.roll_attack()
            print(f"{unit.name} scores {hit} hit(s)")

    for _ in range(2):
        for unit in units:
            unit.take_damage(1)
            print(f"{unit} is still alive." if unit.isalive() else f"{unit} is dead.")


def main():
    """
    Entry point for the game. Initialize and start the Risk game.
    """
    print("\n=======================")
    print("     Welcome to Risk!    ")
    print("=======================\n")

    # TODO: Change the player's name
    game = Risk("LILNOODL")
    game.play()


# Run the game
if __name__ == "__main__":
    main()
    #test()
