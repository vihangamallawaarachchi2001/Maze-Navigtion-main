#!/usr/bin/env python3
"""Render the six explicitly separated Markdown report pages using ReportLab."""
from pathlib import Path
import re
from xml.sax.saxutils import escape
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Image, Table, TableStyle

ROOT = Path(__file__).resolve().parents[1]
styles = getSampleStyleSheet()
styles['BodyText'].fontSize = 10
styles['BodyText'].leading = 14
styles['Heading1'].fontSize = 18
styles['Heading2'].fontSize = 13

def inline(text):
    text = escape(text)
    text = re.sub(r'`([^`]+)`', r'<font face="Courier">\1</font>', text)
    return re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', text)

def main():
    source = ROOT / 'deliverables/final_report.md'
    story = []
    for n, page in enumerate(source.read_text().split('<!-- pagebreak -->')):
        if n:
            story.append(PageBreak())
        lines = page.strip().splitlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                story.append(Spacer(1, 6)); i += 1; continue
            image = re.fullmatch(r'!\[([^]]*)\]\(([^)]+)\)', line)
            if image:
                asset = Image(str(source.parent / image[2]))
                scale = min(6.5*inch/asset.imageWidth, 2.7*inch/asset.imageHeight)
                asset.drawWidth = asset.imageWidth*scale; asset.drawHeight = asset.imageHeight*scale
                story.extend([asset, Paragraph(inline(image[1]), styles['Caption'] if 'Caption' in styles else styles['BodyText'])])
            elif line.startswith('|'):
                rows=[]
                while i < len(lines) and lines[i].strip().startswith('|'):
                    row=lines[i].strip()
                    if not re.fullmatch(r'[| :\-]+',row):
                        rows.append([Paragraph(inline(c.strip()),styles['BodyText']) for c in row.strip('|').split('|')])
                    i+=1
                table=Table(rows,repeatRows=1,hAlign='LEFT')
                table.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e4eaf0')),('GRID',(0,0),(-1,-1),0.3,colors.grey),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),5),('RIGHTPADDING',(0,0),(-1,-1),5)]))
                story.append(table);continue
            elif line.startswith('# '):
                story.append(Paragraph(inline(line[2:]),styles['Heading1']))
            elif line.startswith('## '):
                story.append(Paragraph(inline(line[3:]),styles['Heading2']))
            else:
                text=[line]
                while i+1<len(lines) and lines[i+1].strip() and not lines[i+1].startswith(('#','|','![')):
                    i+=1;text.append(lines[i].strip())
                story.append(Paragraph(inline(' '.join(text)),styles['BodyText']))
            i+=1
    def footer(canvas,doc):
        canvas.setFont('Helvetica',9);canvas.drawRightString(7.5*inch,0.45*inch,f'IE4060 Assignment 02 | {doc.page}')
    doc=SimpleDocTemplate(str(source.with_suffix('.pdf')),pagesize=(8.27*inch,11.69*inch),rightMargin=0.7*inch,leftMargin=0.7*inch,topMargin=0.6*inch,bottomMargin=0.7*inch,title='IE4060 Assignment 02 — Verified Robotics Project')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    print(source.with_suffix('.pdf'))

if __name__=='__main__':
    main()
