import math, sys, pygame


class TextBox:

    def __init__(self, rect, size = 18, color = (255, 255, 255)):
        self.rect = rect
        self.font = pygame.font.SysFont('monospace', size)
        self.color = color

    def set_text(self, text):
        self.text = text

    def render(self, surface):
        self.cut_lines()
        spacing =self.font.size("A")[1] + 10
        offset = 0

        for line in self.lines:
            position = (self.rect.top, self.rect.left + offset)
            text = self.font.render(line, 0, self.color)
            surface.blit(text, position)
            offset += spacing


    def cut_lines(self):
        lines = []
        # Split at newline chars
        raw_lines = self.text.splitlines()

        for raw_line in raw_lines:
            # Until we consume the entire line...
            while raw_line:
                line = self.cut_line(raw_line)

                lines.append(line)
                raw_line = raw_line[len(line):]

        self.lines = lines

    def cut_line(self, text):
        # Actual size of complete text string
        width, height = self.font.size(self.text)
        # Approximate fraction of text we can handle
        fraction = self.rect.width / width
        # Estimated number of chars we can handle
        chars = int(fraction * len(self.text))
        # We need at least one character on each line
        if chars < 1:
            chars = 1

        width, height = self.font.size(self.text[0:chars])
        if width < self.rect.width:
            # Try adding more characters
            while width < self.rect.width:
                if chars >= len(self.text):
                    # This is the last line and won't be a full line
                    break
                width, height = self.font.size(self.text[0:chars + 1])
                if width <= self.rect.width:
                    # We can fit an extra character
                    chars += 1
                else:
                    # We can't fit the extra character
                    break
        elif width > self.rect.width:
            # Remove some characters
            while width > self.rect.width:
                if chars == 1:
                    # Print one char even though it'll be outside our bounds
                    break
                # We need to remove one character
                chars -= 1
                width, height = self.font.size(self.text[0:chars - 1])

        # Return the trimmed line
        return text[0:chars]
