// @ts-nocheck
import {visit} from 'unist-util-visit';

// Rehype plugin to inject the ChatBot component
export default function rehypeChatBot() {
  return (tree) => {
    visit(tree, 'element', (node) => {
      if (node.tagName === 'body') {
        // Add the ChatBot component at the end of the body
        node.children.push({
          type: 'element',
          tagName: 'div',
          properties: {id: 'chatbot-container'},
          children: [{
            type: 'element',
            tagName: 'div',
            properties: {dangerouslySetInnerHTML: {__html: '<div id="chatbot-root"></div>'}},
          }]
        });
      }
    });
  };
}